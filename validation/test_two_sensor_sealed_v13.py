"""Pure pre-access regression tests for the one-shot V13 sealed validator.

The suite never decodes, samples, or evaluates sealed prescribed-GRF values and
never executes OpenSim.  Whole-file hashes may be checked for source identity.
It freezes lineage, candidate roles and geometry, temporal guards, station
accounting, decision semantics, diagnostics, and receipt behavior pre-run.
"""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_ROOT / "two_sensor_sealed_protocol_v13.json"
LOCK_PATH = VALIDATION_ROOT / "two_sensor_sealed_candidate_lock_v13.json"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V13 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import validate_two_sensor_sealed_v13 as subject  # noqa: E402


@dataclass(frozen=True)
class _SetupStub:
    t_start: float = 0.0
    t_end: float = 0.0


class TwoSensorSealedV13Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)
        cls.lock = subject.load_and_validate_lock(LOCK_PATH)
        cls.base, cls.candidates, cls.geometry = subject.build_frozen_candidates(
            cls.protocol, cls.lock
        )

    def _temporary_json(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "payload.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _protocol_payload() -> dict[str, Any]:
        return json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))

    @staticmethod
    def _row(candidate_id: str, *, selectable: bool, hs_error_s: float) -> dict[str, Any]:
        return {
            "candidate_id": candidate_id,
            "selectable": selectable,
            "reference_hs_count": subject.EXPECTED_REFERENCE_HS,
            "reference_to_count": subject.EXPECTED_REFERENCE_TO,
            "predicted_hs_count": subject.EXPECTED_REFERENCE_HS,
            "predicted_to_count": subject.EXPECTED_REFERENCE_TO,
            "observed_valid_cycle_count": subject.EXPECTED_CYCLES,
            "precision": 1.0,
            "recall": 1.0,
            "max_abs_hs_error_s": hs_error_s,
            "max_abs_toe_off_error_s": 0.040,
            "confirmed_fsm_stance_f1": 0.98,
            "confirmed_fsm_stance_iou": 0.95,
            "transfer_both_latches_off_sample_count": 0,
            "incomplete_heel_to_forefoot_transfer_count": 0,
            "to_candidates_before_min_stance_count": 0,
            "invalid_or_timeout_transition_count": 0,
            "unaccepted_sensor_gait_event_count": 0,
            "forbidden_phase_mismatch_count": 0,
            "unknown_fsm_phase_samples": 0,
            "minimum_causal_toe_clear_before_next_hs_onset_s": 0.040,
            "causal_swing_interval_count": subject.EXPECTED_CYCLES,
            "confirmation_latency_in_range": True,
            "exact_hs_to_toe_off_to_hs_order_and_cycle_count": True,
            "mesh_geometry_pre_gate_ok": True,
            "global_mean_signed_hs_error_s": -0.010,
            "global_mean_signed_toe_off_error_s": 0.020,
        }

    def test_protocol_lock_and_sources_are_frozen_before_access(self) -> None:
        self.assertEqual(self.protocol["schema_version"], 13)
        self.assertTrue(self.protocol["frozen_before_execution"])
        self.assertEqual(self.protocol["objective"], subject.OBJECTIVE)
        self.assertEqual(
            self.protocol["interpretation_limits"], subject.INTERPRETATION_LIMITS
        )
        self.assertEqual(
            self.protocol["_protocol_sha256"], subject.v1._sha256(PROTOCOL_PATH)
        )
        self.assertEqual(self.lock["lock_id"], subject.LOCK_ID)
        self.assertTrue(self.lock["frozen_before_sealed_access"])
        self.assertEqual(self.lock["_lock_sha256"], subject.v1._sha256(LOCK_PATH))
        self.assertEqual(
            set(self.protocol["sources"]), set(subject.REQUIRED_SOURCE_PATHS)
        )
        for label, record in self.protocol["sources"].items():
            path = subject.v1.resolve_repo_path(record["path"]).resolve()
            self.assertEqual(record["path"], subject.REQUIRED_SOURCE_PATHS[label])
            self.assertTrue(path.is_file(), label)
            self.assertEqual(record["sha256"], subject.v1._sha256(path), label)

    def test_scope_binds_one_primary_and_two_nondecision_pairs(self) -> None:
        self.assertEqual(
            [item.candidate_id for item in self.candidates],
            [subject.PRIMARY_ID, subject.SENSITIVITY_ID, subject.V9_ID],
        )
        self.assertEqual(sum(item.selectable for item in self.candidates), 1)
        primary, sensitivity, v9 = self.candidates
        self.assertTrue(primary.selectable)
        self.assertFalse(sensitivity.selectable)
        self.assertFalse(v9.selectable)
        self.assertEqual(primary.role, "single_frozen_primary_sealed_candidate")
        self.assertEqual(sensitivity.role, "nonselectable_sensitivity_only")
        self.assertEqual(v9.role, "nonselectable_heel_off_diagnostic_only")
        np.testing.assert_array_equal(
            primary.heel_location, self.lock["primary"]["heel_location_m"]
        )
        np.testing.assert_array_equal(
            primary.forefoot_location, self.lock["primary"]["toe_location_m"]
        )
        np.testing.assert_array_equal(
            sensitivity.forefoot_location,
            self.lock["sensitivity"]["toe_location_m"],
        )
        np.testing.assert_array_equal(
            v9.heel_location,
            self.lock["historical_comparator"]["heel_location_m"],
        )
        self.assertTrue(self.geometry["only_primary_is_selectable"])
        self.assertTrue(self.geometry["radii_unchanged"])

    def test_sampling_contract_is_five_detector_plus_eight_primary(self) -> None:
        sampler, pairs, profiles = subject.v11._sampling_bundle(
            self.base,
            self.candidates,
            stage_label="v13_pure_test",
            expected_detector_stations=subject.DETECTOR_STATIONS,
        )
        self.assertEqual(subject.DETECTOR_STATIONS, 5)
        self.assertEqual(subject.PRIMARY_LOAD_SPHERES, 8)
        self.assertEqual(subject.TOTAL_STATIONS, 13)
        self.assertEqual(len(sampler.spheres), 5)
        self.assertEqual(len({sphere.name for sphere in sampler.spheres}), 5)
        self.assertEqual(set(pairs), {item.candidate_id for item in self.candidates})
        self.assertEqual(set(profiles), set(pairs))
        self.assertTrue(all(len(profile.spheres) == 2 for profile in profiles.values()))
        self.assertEqual(self.protocol["sampling"], subject._expected_sampling())

    def test_protocol_rejects_scope_gate_and_source_hash_drift(self) -> None:
        frozen = self._protocol_payload()

        scope_drift = copy.deepcopy(frozen)
        scope_drift["candidate_scope"]["primary_candidate_id"] = subject.SENSITIVITY_ID
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_json(scope_drift))

        gate_drift = copy.deepcopy(frozen)
        gate_drift["fine_gate_1ms"]["max_abs_hs_error_s"] = 0.051
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_json(gate_drift))

        source_drift = copy.deepcopy(frozen)
        source_drift["sources"]["v12_manifest"]["sha256"] = "0" * 64
        with self.assertRaises(subject.ProtocolError):
            subject.load_and_validate_protocol(self._temporary_json(source_drift))

    def test_cli_lock_must_be_exactly_the_protocol_bound_file(self) -> None:
        subject._validate_lock_binding(self.protocol, self.lock)
        duplicate = self._temporary_json(
            json.loads(LOCK_PATH.read_text(encoding="utf-8"))
        )
        duplicate_lock = subject.load_and_validate_lock(duplicate)
        with self.assertRaises(subject.ProtocolError):
            subject._validate_lock_binding(self.protocol, duplicate_lock)

    def test_synthetic_time_grid_starts_at_100_and_never_leaves_sealed(self) -> None:
        heel_strike = np.linspace(100.5, 154.5, subject.EXPECTED_REFERENCE_HS)
        toe_off = 0.5 * (heel_strike[:-1] + heel_strike[1:])
        events = {"heel_strike": heel_strike, "toe_off": toe_off}
        with (
            patch.object(subject.v1, "read_setup_xml", return_value=_SetupStub()),
            patch.object(
                subject.v1,
                "_reference_events_from_prescribed_grf",
                return_value=(events, {"synthetic": True}),
            ),
        ):
            digests = []
            for cadence in (subject.PRIMARY_DT_S, subject.FINE_DT_S):
                _setup, observed, access, times = subject._reference_bundle(
                    self.protocol, sample_dt_s=cadence
                )
                self.assertEqual(float(times[0]), subject.SEALED_START_S)
                self.assertTrue(np.all(times >= subject.SEALED_START_S))
                self.assertTrue(np.all(times < subject.SEALED_END_S))
                self.assertEqual(len(observed["heel_strike"]), 49)
                self.assertEqual(len(observed["toe_off"]), 48)
                self.assertEqual(access["samples_below_100_s"], 0)
                self.assertEqual(access["samples_at_or_after_155p045_s"], 0)
                self.assertTrue(access["sealed_block_opened"])
                digests.append(access["reference_event_sha256"])
            self.assertEqual(digests[0], digests[1])

        boundary_bad = {
            "heel_strike": np.concatenate((heel_strike[:-1], [155.0])),
            "toe_off": toe_off,
        }
        with (
            patch.object(subject.v1, "read_setup_xml", return_value=_SetupStub()),
            patch.object(
                subject.v1,
                "_reference_events_from_prescribed_grf",
                return_value=(boundary_bad, {"synthetic": True}),
            ),
        ):
            with self.assertRaises(subject.ProtocolError):
                subject._reference_bundle(
                    self.protocol, sample_dt_s=subject.PRIMARY_DT_S
                )

    def test_primary_alone_determines_pass_fail_without_fallback(self) -> None:
        passing_primary = self._row(
            subject.PRIMARY_ID, selectable=True, hs_error_s=0.040
        )
        failing_sensitivity = self._row(
            subject.SENSITIVITY_ID, selectable=False, hs_error_s=0.060
        )
        failing_v9 = self._row(subject.V9_ID, selectable=False, hs_error_s=0.070)
        rows = {
            label: [
                copy.deepcopy(passing_primary),
                copy.deepcopy(failing_sensitivity),
                copy.deepcopy(failing_v9),
            ]
            for label in ("runtime_10ms", "fine_1ms")
        }
        passed = subject.decide_primary(rows, self.protocol)
        self.assertTrue(passed["ok"])
        self.assertFalse(passed["selector_used"])
        self.assertFalse(passed["fallback_used"])

        for label in rows:
            rows[label][0]["max_abs_hs_error_s"] = 0.060
            rows[label][1]["max_abs_hs_error_s"] = 0.020
        failed = subject.decide_primary(rows, self.protocol)
        self.assertFalse(failed["ok"])
        self.assertFalse(failed["sensitivity_can_rescue_primary"])

    def test_heel_off_cadence_audit_is_separate_and_fail_closed(self) -> None:
        cycles_10 = [
            {"cycle_index": index, "heel_off_s": 101.0 + index}
            for index in range(subject.EXPECTED_CYCLES)
        ]
        cycles_1 = [
            {"cycle_index": index, "heel_off_s": 100.991 + index}
            for index in range(subject.EXPECTED_CYCLES)
        ]
        details = {
            "runtime_10ms": {
                subject.PRIMARY_ID: {
                    "heel_to_forefoot_transfer": {"cycles": cycles_10}
                }
            },
            "fine_1ms": {
                subject.PRIMARY_ID: {
                    "heel_to_forefoot_transfer": {"cycles": cycles_1}
                }
            },
        }
        audit = subject._heel_off_cross_cadence(details, subject.PRIMARY_ID)
        self.assertTrue(audit["ok"])
        self.assertEqual(audit["cycles_above_tolerance"], 0)
        details["fine_1ms"][subject.PRIMARY_ID]["heel_to_forefoot_transfer"][
            "cycles"
        ][0]["heel_off_s"] = 100.95
        audit = subject._heel_off_cross_cadence(details, subject.PRIMARY_ID)
        self.assertFalse(audit["ok"])
        self.assertEqual(audit["cycles_above_tolerance"], 1)
        self.assertEqual(
            audit["role"],
            "promotion_readiness_diagnostic_not_sealed_event_gate",
        )

    def test_receipt_precedes_access_and_existing_paths_fail_no_clobber(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "sealed"
            plot_dir = root / "plot"
            receipt = subject._write_receipt(
                output,
                self.protocol,
                self.lock,
                (subject.PRIMARY_ID, subject.SENSITIVITY_ID, subject.V9_ID),
            )
            payload = json.loads(receipt.read_text(encoding="utf-8"))
            self.assertTrue(payload["sealed_access_started"])
            self.assertEqual(payload["primary_candidate_id"], subject.PRIMARY_ID)
            self.assertTrue(
                payload["no_rerun_without_new_explicit_recovery_authorization"]
            )
            with self.assertRaises(subject.v11.NoClobberError):
                subject.v11._preflight_no_clobber(output, plot_dir)
            self.assertFalse(plot_dir.exists())

    def test_cli_rerun_preserves_existing_output_byte_for_byte(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "consumed"
            plot_dir = root / "unused_plot"
            output.mkdir()
            marker = output / "manifest.json"
            marker.write_bytes(b"already-consumed\n")
            before = {
                path.relative_to(output).as_posix(): path.read_bytes()
                for path in output.rglob("*")
                if path.is_file()
            }
            exit_code = subject.main(
                [
                    "--protocol",
                    str(PROTOCOL_PATH),
                    "--candidate-lock",
                    str(LOCK_PATH),
                    "--output-dir",
                    str(output),
                    "--plot-dir",
                    str(plot_dir),
                ]
            )
            after = {
                path.relative_to(output).as_posix(): path.read_bytes()
                for path in output.rglob("*")
                if path.is_file()
            }
            self.assertEqual(exit_code, 2)
            self.assertEqual(after, before)
            self.assertFalse((output / "failure.json").exists())
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
