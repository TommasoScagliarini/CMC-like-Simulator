"""Pure pre-access tests for the frozen V21 trial-08 one-shot validator.

The suite never opens the trial-08 preprocessing lock, IK motion, prescribed
GRF, ExternalLoads, or event oracle and never samples OpenSim.  It verifies the
freeze lineage, immutable write behavior, fixed one-shot destination, access
ordering, lattice, and event-time contract using only public metadata and
synthetic values.
"""

from __future__ import annotations

import copy
import base64
import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_binary_phase_detector_v21_trial08 as freeze  # noqa: E402
import validate_binary_phase_detector_v21_trial08_one_shot as subject  # noqa: E402


class V21Trial08PreAccessTests(unittest.TestCase):
    def _temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def test_freeze_build_does_not_open_declared_trial08_inputs(self) -> None:
        original_strict = freeze._strict_json
        forbidden = {
            str(record["path"])
            for record in freeze.TRIAL08_DECLARATIONS.values()
        }
        opened: list[str] = []

        def guarded(path: Path, expected_sha256: str):
            relative = path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
            self.assertNotIn(relative, forbidden)
            opened.append(relative)
            return original_strict(path, expected_sha256)

        with patch.object(freeze, "_strict_json", side_effect=guarded):
            payload = freeze.build_freeze_payload()

        self.assertTrue(opened)
        self.assertFalse(payload["trial"]["scientifically_virgin"])
        self.assertTrue(
            payload["trial"]["protocol_declaration_check"][
                "declared_without_opening"
            ]
        )
        self.assertFalse(payload["data_governance"]["trial08_opened_by_this_freeze"])
        metadata_access = payload["data_governance"][
            "current_cycle_pre_freeze_nonperformance_metadata_access"
        ]
        self.assertTrue(metadata_access["occurred"])
        self.assertTrue(metadata_access["scientific_performance_blinding_preserved"])
        self.assertFalse(metadata_access["detector_trace_or_metrics_read"])
        self.assertFalse(
            set(forbidden)
            & {
                str(record["path"])
                for record in payload["sources"].values()
            }
        )

    def test_freeze_binds_exact_candidate_and_limited_claim(self) -> None:
        payload = freeze.build_freeze_payload()
        self.assertEqual(payload["candidate"]["candidate_id"], freeze.CANDIDATE_ID)
        self.assertEqual(payload["candidate"]["profile"]["sha256"], freeze.PROFILE_SHA256)
        self.assertEqual(payload["candidate"]["candidate_count_after_freeze"], 1)
        self.assertEqual(payload["gate"]["required_unit_count"], 8)
        self.assertEqual(payload["gate"]["required_view_count"], 4)
        self.assertFalse(payload["post_pass_scope"]["runtime_promotion_allowed"])
        self.assertFalse(payload["post_pass_scope"]["training_promotion_allowed"])
        self.assertEqual(payload["post_pass_scope"]["next_independent_gates"], ["05", "06"])
        self.assertEqual(payload["data_governance"]["protected_trials_opened"], [])
        self.assertEqual(payload["data_governance"]["reserve_trials_opened"], [])

    def test_preflight_does_not_call_semantic_input_verifier(self) -> None:
        temporary = self._temporary_dir()
        lock_path = temporary / "freeze.json"
        lock_path.write_text(
            json.dumps(freeze.build_freeze_payload(), allow_nan=False),
            encoding="utf-8",
        )
        with (
            patch.object(freeze, "FREEZE_PATH", lock_path),
            patch.object(subject.platform, "system", return_value="Darwin"),
            patch.object(subject.platform, "machine", return_value="arm64"),
            patch.object(
                subject,
                "_verify_declared_input",
                side_effect=AssertionError("semantic trial input touched"),
            ),
        ):
            result = subject.preflight_unopened()
        self.assertEqual(
            result["status"], "V21_TRIAL08_ONE_SHOT_PREFLIGHT_READY_UNOPENED"
        )
        self.assertFalse(result["trial08_semantic_access"])

    def test_semantic_helpers_fail_before_opening_receipts(self) -> None:
        temporary = self._temporary_dir()
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", temporary / "ledger.json"),
            patch.object(freeze, "OUTPUT_DIR", temporary / "run"),
        ):
            with self.assertRaises(subject.V21Trial08Error):
                subject._validate_trial08_replay_inputs({})
            with self.assertRaises(subject.V21Trial08Error):
                subject._load_oracle({})
            with self.assertRaises(subject.V21Trial08Error):
                subject._acquire_trace(
                    {}, temporary / "profile.json", plugin_preloaded=False
                )
            with self.assertRaises(subject.V21Trial08Error):
                subject._verify_declared_input(
                    {"path": "forbidden", "sha256": "x", "size_bytes": 0},
                    label="synthetic",
                )

    def test_execute_publishes_both_receipts_before_semantic_input_access(self) -> None:
        temporary = self._temporary_dir()
        output_dir = temporary / "runs" / "fixed"
        ledger_path = temporary / "execution_ledger.json"
        freeze_path = temporary / "freeze.json"
        freeze_path.write_text("{}\n", encoding="utf-8")
        fake_lock = {
            "candidate": {"profile": {"path": "unused"}},
            "data_governance": {
                "current_cycle_pre_freeze_nonperformance_metadata_access": {
                    "occurred": True,
                    "detector_trace_or_metrics_read": False,
                }
            },
        }
        fake_preflight = {
            "freeze_lock": {
                "path": "validation/fake_freeze.json",
                "sha256": subject.sha256_file(freeze_path),
                "size_bytes": freeze_path.stat().st_size,
            },
            "lock": fake_lock,
            "platform": {"system": "Darwin", "machine": "arm64"},
            "common_environment": {
                "checks": {"pass": True},
                "plugin_loaded_in_current_process": True,
            },
        }
        order: list[str] = []

        def semantic_inputs(_lock):
            self.assertTrue(ledger_path.is_file())
            self.assertTrue((output_dir / subject.ACCESS_RECEIPT_NAME).is_file())
            order.append("semantic")
            return {"paths": {}, "source_records": {}}

        def acquire(_inputs, _profile, **_kwargs):
            order.append("trace")
            return {"trace": True}

        def oracle(_lock):
            order.append("oracle")
            return {"oracle": True}

        def evaluate(_lock, _trace, _oracle):
            order.append("evaluate")
            return {"pass": True}

        def persist(**_kwargs):
            order.append("persist")
            return {"decision": {"pass": True}}

        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger_path),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
            patch.object(freeze, "FREEZE_PATH", freeze_path),
            patch.object(subject, "preflight_unopened", return_value=fake_preflight),
            patch.object(
                subject,
                "_validate_trial08_replay_inputs",
                side_effect=semantic_inputs,
            ),
            patch.object(subject, "_verify_source_record", return_value=temporary / "p"),
            patch.object(subject, "_acquire_trace", side_effect=acquire),
            patch.object(subject, "_load_oracle", side_effect=oracle),
            patch.object(subject, "_evaluate", side_effect=evaluate),
            patch.object(subject, "_write_result_artifacts", side_effect=persist),
        ):
            result = subject.execute_one_shot()

        self.assertTrue(result["decision"]["pass"])
        self.assertEqual(order, ["semantic", "oracle", "trace", "evaluate", "persist"])
        ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
        receipt = json.loads(
            (output_dir / subject.ACCESS_RECEIPT_NAME).read_text(encoding="utf-8")
        )
        self.assertEqual(ledger, receipt)
        self.assertTrue(ledger["stage_consumed"])
        self.assertFalse(ledger["rerun_allowed"])

    def test_atomic_writer_is_strict_no_clobber_and_leaves_no_temporary(self) -> None:
        temporary = self._temporary_dir()
        path = temporary / "receipt.json"
        subject._write_json_exclusive(path, {"value": 1.0})
        original = path.read_bytes()
        with self.assertRaises(FileExistsError):
            subject._write_json_exclusive(path, {"value": 2.0})
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(list(temporary.glob(".*.tmp")), [])

        nonfinite = temporary / "nonfinite.json"
        with self.assertRaises(ValueError):
            subject._write_json_exclusive(nonfinite, {"value": math.nan})
        self.assertFalse(nonfinite.exists())
        self.assertEqual(list(temporary.glob(".*.tmp")), [])

    def test_fixed_lattice_is_unique_monotonic_and_exact(self) -> None:
        times = subject._time_grid()
        self.assertEqual(times.size, freeze.EXPECTED_SAMPLE_COUNT)
        self.assertAlmostEqual(float(times[0]), freeze.TRIAL_INTERVAL_S[0], places=12)
        self.assertAlmostEqual(float(times[-1]), freeze.TRIAL_INTERVAL_S[1], places=9)
        self.assertTrue(bool((times[1:] > times[:-1]).all()))
        self.assertTrue(bool((abs((times[1:] - times[:-1]) - 0.001) <= 1e-12).all()))

    def test_packed_binary_trace_roundtrips_channel_order_and_hash(self) -> None:
        heel = np.asarray([False, True, True, False, True], dtype=bool)
        toe = np.asarray([True, False, True, False, False], dtype=bool)
        payload = subject._pack_binary_bits(heel, toe)
        raw = base64.b64decode(payload["packed_bits_base64"], validate=True)
        self.assertEqual(hashlib.sha256(raw).hexdigest(), payload["bit_trace_sha256"])
        unpacked = np.unpackbits(
            np.frombuffer(raw, dtype=np.uint8), bitorder="little"
        )[: 2 * heel.size].reshape(-1, 2)
        np.testing.assert_array_equal(unpacked[:, 0].astype(bool), heel)
        np.testing.assert_array_equal(unpacked[:, 1].astype(bool), toe)

    def test_tampered_gate_or_post_pass_scope_is_rejected(self) -> None:
        for section in ("gate", "post_pass_scope"):
            with self.subTest(section=section):
                temporary = self._temporary_dir()
                payload = freeze.build_freeze_payload()
                payload[section] = copy.deepcopy(payload[section])
                first_key = next(iter(payload[section]))
                payload[section][first_key] = "tampered"
                lock_path = temporary / f"tampered_{section}.json"
                lock_path.write_text(
                    json.dumps(payload, allow_nan=False), encoding="utf-8"
                )
                with patch.object(freeze, "FREEZE_PATH", lock_path):
                    with self.assertRaises(subject.V21Trial08Error):
                        subject._validate_freeze_lock()

    def test_full_event_contract_accepts_only_exact_debounce_and_delivery(self) -> None:
        good = [
            {
                "event": "heel_strike",
                "event_time_s": 1.000,
                "confirmed_time_s": 1.005,
                "delivered_time_s": 1.010,
            },
            {
                "event": "toe_off",
                "event_time_s": 1.500,
                "confirmed_time_s": 1.505,
                "delivered_time_s": 1.510,
            },
        ]
        self.assertTrue(subject._full_event_contract(good)["pass"])
        bad = copy.deepcopy(good)
        bad[0]["confirmed_time_s"] = 1.006
        self.assertFalse(subject._full_event_contract(bad)["pass"])
        bad = copy.deepcopy(good)
        bad[0]["delivered_time_s"] = 1.016
        self.assertFalse(subject._full_event_contract(bad)["pass"])

    def test_failure_receipt_is_terminal_and_no_clobber(self) -> None:
        temporary = self._temporary_dir()
        output_dir = temporary / "run"
        output_dir.mkdir()
        ledger = temporary / "ledger.json"
        opening = {
            "process_id": subject.os.getpid(),
            "candidate_id": freeze.CANDIDATE_ID,
            "trial_id": freeze.TRIAL_ID,
        }
        ledger.write_text(json.dumps(opening) + "\n", encoding="utf-8")
        (output_dir / subject.ACCESS_RECEIPT_NAME).write_text(
            json.dumps(opening) + "\n", encoding="utf-8"
        )
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            subject._write_failure_after_open(RuntimeError("synthetic failure"))
            path = output_dir / subject.FAILURE_NAME
            first = path.read_bytes()
            subject._write_failure_after_open(RuntimeError("second failure"))
        self.assertEqual(path.read_bytes(), first)
        payload = json.loads(first)
        self.assertTrue(payload["stage_consumed"])
        self.assertFalse(payload["rerun_allowed"])
        self.assertEqual(payload["next_stage"], "STOP_NO_RETRY_NO_RESELECT_NO_RETUNE")

    def test_evidence_manifest_is_bound_before_terminal_decision(self) -> None:
        temporary = self._temporary_dir()
        output_dir = temporary / "run"
        output_dir.mkdir()
        ledger = temporary / "ledger.json"
        opening = {"process_id": subject.os.getpid()}
        ledger.write_text(json.dumps(opening) + "\n", encoding="utf-8")
        (output_dir / subject.ACCESS_RECEIPT_NAME).write_text(
            json.dumps(opening) + "\n", encoding="utf-8"
        )
        trace = {
            "time_s": np.asarray([0.0, 0.001]),
            "heel": np.asarray([False, True]),
            "toe": np.asarray([False, False]),
            "sample_count": 2,
            "bit_trace_sha256": "a" * 64,
            "time_trace_sha256": "b" * 64,
            "packed_bits_base64": "AA==",
            "packed_bits_size_bytes": 1,
            "baseline": {},
            "clearance_range_m": {},
        }
        evaluation = {
            "pass": True,
            "unit_count": 8,
            "unit_pass_count": 8,
            "two_sensor_channel_gate": {"pass": True},
            "parity": {"pass": True},
            "full_event_contract": {},
            "fsm_contract_binding": {"pass": True},
            "terminal_pending_state_gate": {"pass": True},
            "units": [],
            "events": {},
            "diagnostics": {},
        }
        preflight = {
            "lock": {
                "trial": {"claim_limit": "internal only"},
                "post_pass_scope": freeze.POST_PASS_SCOPE,
            },
            "freeze_lock": {"path": "validation/freeze.json", "sha256": "c" * 64},
            "platform": {"system": "Darwin", "machine": "arm64"},
            "common_environment": {"checks": {"pass": True}},
        }
        oracle = {
            "checks": {"pass": True},
            "view_integrity": {"pass": True},
            "source": {"path": "validation/oracle.json", "sha256": "d" * 64},
        }
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            result = subject._write_result_artifacts(
                opening=opening,
                preflight=preflight,
                replay_inputs={"source_records": {}},
                trace=trace,
                oracle=oracle,
                evaluation=evaluation,
            )
        manifest_path = output_dir / subject.MANIFEST_NAME
        decision_path = output_dir / subject.DECISION_NAME
        self.assertTrue(manifest_path.is_file())
        self.assertTrue(decision_path.is_file())
        decision = json.loads(decision_path.read_text(encoding="utf-8"))
        self.assertEqual(
            decision["evidence_manifest"]["sha256"],
            subject.sha256_file(manifest_path),
        )
        self.assertEqual(result["terminal_decision"]["sha256"], subject.sha256_file(decision_path))
        self.assertFalse((output_dir / subject.FAILURE_NAME).exists())

    def test_preexisting_terminal_run_is_never_mutated_by_later_process(self) -> None:
        temporary = self._temporary_dir()
        output_dir = temporary / "run"
        output_dir.mkdir()
        ledger = temporary / "ledger.json"
        opening = {
            "process_id": subject.os.getpid() + 1000,
            "candidate_id": freeze.CANDIDATE_ID,
            "trial_id": freeze.TRIAL_ID,
        }
        ledger.write_text(json.dumps(opening) + "\n", encoding="utf-8")
        (output_dir / subject.ACCESS_RECEIPT_NAME).write_text(
            json.dumps(opening) + "\n", encoding="utf-8"
        )
        decision = output_dir / subject.DECISION_NAME
        decision.write_text('{"pass":true}\n', encoding="utf-8")
        before = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        with (
            patch.object(freeze, "EXECUTION_LEDGER_PATH", ledger),
            patch.object(freeze, "OUTPUT_DIR", output_dir),
        ):
            subject._write_failure_after_open(RuntimeError("later process"))
        after = {path.name: path.read_bytes() for path in output_dir.iterdir()}
        self.assertEqual(after, before)
        self.assertFalse((output_dir / subject.FAILURE_NAME).exists())


if __name__ == "__main__":
    unittest.main()
