"""Pure tests for the corrected V2 dual-stream integrity control."""

from __future__ import annotations

import copy
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_DIR = REPO_ROOT / "validation"
PROTOCOL_PATH = VALIDATION_DIR / "two_sensor_dual_stream_prescribed_protocol_v2.json"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure dual-stream V2 test touched opensim.{name}")


with patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import diagnose_two_sensor_dual_stream_prescribed_v2 as subject  # noqa: E402


class TwoSensorDualStreamPrescribedV2Test(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = subject.load_and_validate_protocol(PROTOCOL_PATH)

    def _temporary_protocol(self, payload: dict[str, Any]) -> Path:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "protocol.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    @staticmethod
    def _trace_inputs() -> dict[str, Any]:
        return {
            "loads": {
                "left_heel": np.asarray([0.0, 0.6, 0.7]),
                "left_toe": np.asarray([0.0, 0.0, 0.8]),
            }
        }

    @staticmethod
    def _replay(candidate_event: str) -> dict[str, Any]:
        return {
            "heel_contact": np.asarray([0.0, 1.0, 1.0]),
            "toe_contact": np.asarray([0.0, 0.0, 1.0]),
            "sensor_edges": [
                {
                    "sensor": "heel",
                    "edge": "contact_on",
                    "event_time_s": 50.01,
                    "confirmed_time_s": 50.04,
                }
            ],
            "candidates": [
                {
                    "event": candidate_event,
                    "event_time_s": 50.01,
                }
            ],
        }

    @staticmethod
    def _comparison_rows() -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for candidate_id in subject.v1.CANDIDATE_IDS:
            for branch_id, load_rejections, valid_cycles, candidate_hash in (
                ("A_detector_load", 3, 0, "state-a"),
                ("B_primary_load", 0, 50, "state-b"),
            ):
                rows.append(
                    {
                        "candidate_id": candidate_id,
                        "branch_id": branch_id,
                        "raw_detector_force_trace_sha256": "same-force",
                        "debounced_detector_latch_trace_sha256": "same-latch",
                        "raw_detector_edge_trace_sha256": "same-edge",
                        "raw_detector_guard_trace_sha256": "same-raw",
                        "fsm_conditioned_candidate_trace_sha256": candidate_hash,
                        "fsm_conditioned_candidate_count": 2,
                        "stance_load_too_low_count": load_rejections,
                        "predicted_hs_count": 51,
                        "predicted_to_count": 50,
                        "observed_valid_cycle_count": valid_cycles,
                        "invalid_reason_counts_json": json.dumps(
                            {"stance_load_too_low": load_rejections}
                        ),
                    }
                )
        return rows

    def test_protocol_inherits_v1_and_freezes_raw_guard_correction(self) -> None:
        correction = self.protocol["guard_trace_correction"]
        self.assertEqual(
            correction["fsm_conditioned_sensor_event_candidates_role"],
            "diagnostic_only",
        )
        self.assertFalse(
            correction["fsm_conditioned_candidate_drift_invalidates_ab"]
        )
        self.assertFalse(correction["accepted_transitions_in_ab_integrity_hash"])
        self.assertEqual(
            self.protocol["candidate_ids"], list(subject.v1.CANDIDATE_IDS)
        )
        self.assertFalse(
            self.protocol["decision_contract"]["profile_promotion_allowed"]
        )

    def test_protocol_rejects_integrity_or_lineage_drift(self) -> None:
        frozen = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
        for mutation in ("integrity", "lineage"):
            payload = copy.deepcopy(frozen)
            if mutation == "integrity":
                payload["guard_trace_correction"][
                    "fsm_conditioned_candidate_drift_invalidates_ab"
                ] = True
            else:
                payload["derived_from"]["invalid_manifest_sha256"] = "0" * 64
            with self.subTest(mutation=mutation), self.assertRaises(
                subject.ProtocolError
            ):
                subject.load_and_validate_protocol(
                    self._temporary_protocol(payload)
                )

    def test_raw_guard_hash_ignores_state_conditioned_candidates(self) -> None:
        inputs = self._trace_inputs()
        trace_a = subject._detector_trace_diagnostics(
            inputs, self._replay("toe_off")
        )
        trace_b = subject._detector_trace_diagnostics(
            inputs, self._replay("heel_strike")
        )
        self.assertEqual(
            trace_a["raw_detector_guard_trace_sha256"],
            trace_b["raw_detector_guard_trace_sha256"],
        )
        self.assertNotEqual(
            trace_a["fsm_conditioned_candidate_trace_sha256"],
            trace_b["fsm_conditioned_candidate_trace_sha256"],
        )
        self.assertNotIn(
            "fsm_conditioned_sensor_event_candidates",
            self.protocol["guard_trace_correction"]["ab_integrity_gate_fields"],
        )

    def test_candidate_drift_is_diagnostic_not_ab_integrity_failure(self) -> None:
        comparisons = subject.compare_branches(self._comparison_rows())
        self.assertTrue(
            all(item["raw_detector_ab_integrity_ok"] for item in comparisons)
        )
        self.assertTrue(
            all(
                not item["fsm_conditioned_candidate_trace_identical"]
                for item in comparisons
            )
        )
        self.assertEqual(
            subject._diagnostic_conclusion(comparisons),
            "PRIMARY_LOAD_REMOVES_STANCE_LOAD_TOO_LOW_FOR_ALL_CANDIDATES",
        )

    def test_real_raw_guard_drift_remains_invalid(self) -> None:
        rows = self._comparison_rows()
        rows[1]["raw_detector_edge_trace_sha256"] = "drift"
        comparisons = subject.compare_branches(rows)
        self.assertFalse(comparisons[0]["raw_detector_ab_integrity_ok"])
        self.assertEqual(
            subject._diagnostic_conclusion(comparisons),
            "INVALID_AB_RAW_DETECTOR_GUARD_STREAM_DRIFT",
        )

    def test_v1_files_and_artifacts_remain_hash_identical(self) -> None:
        expected = {
            "validation/diagnose_two_sensor_dual_stream_prescribed.py": (
                "e9376dd841456b77af6a6bda97895143e3f8d70023d4282f9a90a2579b37a1a1"
            ),
            "validation/two_sensor_dual_stream_prescribed_protocol.json": (
                "98667695d844fef23a6da242ddec2432bffe46e944b59412416a02beff4e56e6"
            ),
            "validation/test_two_sensor_dual_stream_prescribed.py": (
                "4de184a2bfc7e3fd3bd3732a1ca9c99194085877026c6236025c3003d37eb59f"
            ),
            (
                "validation/two_sensor_dual_stream_diagnostic_runs/"
                "2026-07-22_ab06_50_100_h02_f70_p315_p320/manifest.json"
            ): "08419eb3f454a264e98f90049788318a081abd3fa164ddbda288029d7637172e",
        }
        for relative, digest in expected.items():
            with self.subTest(relative=relative):
                self.assertEqual(subject.v1.v1._sha256(REPO_ROOT / relative), digest)

    def test_defaults_are_new_versioned_paths_and_no_clobber(self) -> None:
        self.assertIn("v2_raw_guard", subject.DEFAULT_OUTPUT_DIR.name)
        self.assertIn("v2_raw_guard", subject.DEFAULT_PLOT_DIR.name)
        self.assertNotEqual(subject.DEFAULT_OUTPUT_DIR, subject.V1_MANIFEST_PATH.parent)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output_dir = root / "occupied-output"
            plot_dir = root / "new-plot"
            output_dir.mkdir()
            sentinel = output_dir / "prior.txt"
            sentinel.write_text("preserve", encoding="utf-8")
            with redirect_stdout(io.StringIO()):
                exit_code = subject.main(
                    [
                        "--protocol",
                        str(PROTOCOL_PATH),
                        "--output-dir",
                        str(output_dir),
                        "--plot-dir",
                        str(plot_dir),
                    ]
                )
            self.assertEqual(exit_code, 2)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve")
            self.assertFalse(plot_dir.exists())


if __name__ == "__main__":
    unittest.main()
