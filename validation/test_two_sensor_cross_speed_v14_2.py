"""Pure, synthetic, and provenance tests for the V14.2 boundary recovery."""

from __future__ import annotations

import copy
import inspect
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest import mock

import numpy as np


VALIDATION_ROOT = Path(__file__).resolve().parent
if str(VALIDATION_ROOT) not in sys.path:
    sys.path.insert(0, str(VALIDATION_ROOT))


class _ForbiddenOpenSim(ModuleType):
    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"pure V14.2 test touched opensim.{name}")


with mock.patch.dict(sys.modules, {"opensim": _ForbiddenOpenSim("opensim")}):
    import sweep_two_sensor_cross_speed_v14_2 as subject  # noqa: E402


def _events(heel: list[float], toe: list[float]) -> dict[str, np.ndarray]:
    return {
        "heel_strike": np.asarray(heel, dtype=float),
        "toe_off": np.asarray(toe, dtype=float),
    }


class BoundaryRecoveryTest(unittest.TestCase):
    def test_parent_v141_failure_and_inventory_are_exact(self) -> None:
        evidence = subject._validate_parent_v141_failure()
        self.assertEqual(
            evidence["status"], "PASS_PARENT_V14_1_PRE_REPLAY_FAILURE_VERIFIED"
        )
        self.assertTrue(all(evidence["checks"].values()))
        self.assertEqual(
            evidence["output_inventory_sha256"],
            subject.EXPECTED_PARENT_OUTPUT_FILES,
        )

    def test_boundary_lineage_is_exact(self) -> None:
        record = subject._validated_boundary_lineage()
        self.assertEqual(record["sha256"], subject.EXPECTED_BOUNDARY_LINEAGE_SHA256)

    def test_protocol_delta_preserves_normalized_inherited_core(self) -> None:
        parent = subject._load_json(subject.PARENT_PROTOCOL)
        child = subject.expected_protocol_payload(require_all_sources=False)
        self.assertEqual(
            subject._normalized_inherited_scientific_core(child),
            subject._normalized_inherited_scientific_core(parent),
        )
        self.assertEqual(child["split"], parent["split"])
        self.assertEqual(child["trials"], parent["trials"])
        self.assertEqual(child["gate_contract"], parent["gate_contract"])
        self.assertEqual(
            child["unit_boundary_handling"][
                "v14_2_prescribed_left_prefix_censor"
            ],
            subject.FROZEN_BOUNDARY_CONTRACT,
        )

    def test_protocol_top_level_delta_is_exact_and_sources_have_no_placeholders(self) -> None:
        parent = subject._load_json(subject.PARENT_PROTOCOL)
        child = subject.expected_protocol_payload(require_all_sources=True)
        changed = {
            key
            for key in set(parent) | set(child)
            if parent.get(key) != child.get(key)
        }
        self.assertEqual(
            changed,
            {
                "protocol_id",
                "stage",
                "objective",
                "interpretation_limits",
                "preprocessing",
                "execution_destination",
                "recovery",
                "sources",
                "unit_boundary_handling",
            },
        )
        self.assertTrue(
            all(
                record["sha256"] != "<FILL_AFTER_SOURCE_EXISTS>"
                for record in child["sources"].values()
            )
        )

    def test_split_holdout_and_reserve_barriers_are_inherited(self) -> None:
        payload = subject.expected_protocol_payload(require_all_sources=False)
        self.assertEqual(payload["split"]["DEVELOPMENT"], ["02", "04", "08"])
        self.assertEqual(payload["split"]["VALIDATION"], ["05"])
        self.assertEqual(payload["split"]["SEALED"], ["06"])
        self.assertEqual(payload["split"]["RESERVE"], ["03", "07"])
        source = inspect.getsource(subject.run_cross_speed_protocol)
        self.assertIn("v14.run_cross_speed_protocol", source)
        self.assertNotIn("RESERVE", inspect.getsource(subject._prepare_trial_hook))

    def test_protocol_loader_rejects_alternate_path_before_read(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            alternate = Path(tmp) / "protocol.json"
            alternate.write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(subject.ProtocolError, "canonical"):
                subject.load_and_validate_protocol(alternate)

    def test_runtime_guard_rejects_mutated_mapping(self) -> None:
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

    def test_dev02_minimum_prefix_is_two_and_leaves_nineteen_cycles(self) -> None:
        heel = [
            11.743851116883118,
            13.497819199999999,
            15.159757641618498,
            *[16.0 + index for index in range(19)],
        ]
        # Preserve strict ordering after the real first three timestamps.
        heel = heel[:3] + [16.2 + index for index in range(19)]
        toe = [
            12.927057035971224,
            14.607070411347516,
            *[(heel[index] + heel[index + 1]) / 2.0 for index in range(2, 21)],
        ]
        raw = _events(heel, toe)
        warmup = copy.deepcopy(raw)
        retained, access = subject.retain_scoreable_plateau_cycles_v142(
            raw,
            plateau_start_s=11.698,
            plateau_end_s=36.0,
            warmup_prescribed_events=warmup,
        )
        self.assertEqual(access["excluded_left_boundary_cycle_count"], 2)
        self.assertAlmostEqual(
            retained["heel_strike"][0], 15.159757641618498, places=12
        )
        self.assertEqual(access["retained_scoreable_cycle_count"], 19)
        self.assertEqual(
            access["selected_prescribed_warmup_cycle"]["closing_hs_s"],
            13.497819199999999,
        )
        self.assertTrue(access["left_retained_right_cycle_identity_ok"])

    def test_one_drop_has_guard_but_no_warmup_cycle(self) -> None:
        raw = _events(
            [11.743851116883118, 13.4978192, 15.159757641618498, 16.5],
            [12.927057035971224, 14.607070411347516, 15.8],
        )
        one_drop_anchor = float(raw["heel_strike"][1])
        self.assertGreater(one_drop_anchor - 11.698, subject.v14.LEFT_CONTEXT_S)
        eligible = subject._eligible_prescribed_warmup_cycles(
            raw,
            cutoff_s=one_drop_anchor - subject.v14.LEFT_CONTEXT_S,
        )
        self.assertEqual(eligible, [])

    def test_first_plateau_anchor_must_exist_in_warmup_reference(self) -> None:
        raw = _events(
            [0.01, 0.5, 1.0, 1.5],
            [0.2, 0.7, 1.2],
        )
        mismatched_warmup = _events(
            [0.01, 0.5, 1.1, 1.6],
            [0.2, 0.8, 1.3],
        )
        with self.assertRaisesRegex(subject.ProtocolError, "no prescribed left prefix"):
            subject.retain_scoreable_plateau_cycles_v142(
                raw,
                plateau_start_s=0.0,
                plateau_end_s=2.0,
                warmup_prescribed_events=mismatched_warmup,
                minimum_cycles=1,
            )

    def test_exact_90ms_guard_is_accepted_without_warmup_requirement(self) -> None:
        raw = _events(
            [0.09, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0],
            [0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5],
        )
        retained, access = subject.retain_scoreable_plateau_cycles_v142(
            raw,
            plateau_start_s=0.0,
            plateau_end_s=12.0,
            warmup_prescribed_events=None,
        )
        self.assertEqual(access["excluded_left_boundary_cycle_count"], 0)
        self.assertEqual(retained["heel_strike"][0], 0.09)
        self.assertIsNone(access["warmup_anchor_membership_verified"])

    def test_right_censor_runs_after_left_prefix_with_exact_accounting(self) -> None:
        raw = _events(
            [0.01, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5],
            [0.2, 0.7, 1.2, 1.7, 2.2, 2.7, 3.2, 3.7, 4.2, 4.7, 5.2, 5.7, 6.2],
        )
        retained, access = subject.retain_scoreable_plateau_cycles_v142(
            raw,
            plateau_start_s=0.0,
            plateau_end_s=6.53,
            warmup_prescribed_events=None,
            right_observation_margin_s=0.06,
            minimum_cycles=10,
        )
        self.assertEqual(access["excluded_left_boundary_cycle_count"], 1)
        self.assertEqual(access["excluded_right_boundary_cycle_count"], 1)
        self.assertEqual(access["retained_scoreable_cycle_count"], 11)
        self.assertEqual(access["raw_complete_cycle_count"], 13)
        self.assertEqual(len(retained["toe_off"]), 11)

    def test_fails_if_boundary_censors_leave_fewer_than_ten_cycles(self) -> None:
        raw = _events(
            [0.01, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0],
            [0.2, 0.7, 1.2, 1.7, 2.2, 2.7, 3.2, 3.7, 4.2, 4.7],
        )
        with self.assertRaises(subject.v14.ProtocolError):
            subject.retain_scoreable_plateau_cycles_v142(
                raw,
                plateau_start_s=0.0,
                plateau_end_s=6.0,
                warmup_prescribed_events=None,
            )

    def test_malformed_excluded_prefix_is_rejected_before_slicing(self) -> None:
        raw = _events(
            [0.01, 0.5, 1.0, 2.0],
            [0.6, 0.7, 1.5],
        )
        with self.assertRaisesRegex(subject.ProtocolError, "HS--TO--HS"):
            subject.retain_scoreable_plateau_cycles_v142(
                raw,
                plateau_start_s=0.0,
                plateau_end_s=3.0,
                warmup_prescribed_events=None,
                minimum_cycles=1,
            )

    def test_boundary_reference_selection_has_no_cadence_input(self) -> None:
        raw = _events(
            [0.01, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5],
            [0.2, 0.7, 1.2, 1.7, 2.2, 2.7, 3.2, 3.7, 4.2, 4.7, 5.2],
        )
        parameters = inspect.signature(
            subject.retain_scoreable_plateau_cycles_v142
        ).parameters
        self.assertNotIn("sample_dt_s", parameters)
        self.assertNotIn("cadence", parameters)
        first = subject.retain_scoreable_plateau_cycles_v142(
            raw,
            plateau_start_s=0.0,
            plateau_end_s=6.0,
            warmup_prescribed_events=None,
            minimum_cycles=10,
        )[1]["reference_event_sha256"]
        second = subject.retain_scoreable_plateau_cycles_v142(
            copy.deepcopy(raw),
            plateau_start_s=0.0,
            plateau_end_s=6.0,
            warmup_prescribed_events=None,
            minimum_cycles=10,
        )[1]["reference_event_sha256"]
        self.assertEqual(first, second)

    def test_trial02_path_never_calls_raw_verification_converter_or_ik(self) -> None:
        source = inspect.getsource(subject._prepare_trial02_from_parent)
        self.assertNotIn("_verify_raw_identity", source)
        self.assertNotIn("convert_trial", source)
        self.assertNotIn("run_ik_from_setup", source)
        self.assertNotIn("finalize_ik_receipt", source)
        receipt_index = source.index("import_receipt = _RAW_WRITE_JSON")
        copy_index = source.index("_copy_exclusive")
        self.assertLess(receipt_index, copy_index)
        self.assertIn(
            '"written_before_parent_product_copy_or_semantic_replay": True',
            source,
        )
        self.assertIn(
            '"prior_hash_only_parent_identity_checks_allowed": True', source
        )

    def test_nontrial02_delegates_to_pinned_v141_prepare(self) -> None:
        with mock.patch.object(
            subject.v141, "_prepare_trial_hook", return_value="delegated"
        ) as prepare:
            observed = subject._prepare_trial_hook(
                {},
                trial_id="04",
                stage="development",
                work_dir=Path("/synthetic"),
                access_receipt=None,
            )
        self.assertEqual(observed, "delegated")
        prepare.assert_called_once()

    def test_real_trial02_copy_and_v141_lock_validation_succeed(self) -> None:
        protocol = subject.expected_protocol_payload(require_all_sources=False)
        protocol["_protocol_path"] = subject.PARENT_PROTOCOL.as_posix()
        protocol["_protocol_sha256"] = subject._sha256(subject.PARENT_PROTOCOL)
        temporary_parent = subject.DEFAULT_OUTPUT_DIR.parent
        temporary_parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_parent) as tmp:
            root = Path(tmp)
            ledger = root / "ledger.json"
            ledger.write_text("{}\n", encoding="utf-8")
            state = subject.v141._RuntimeState(
                protocol, root, {}, {}, {}, {}, {}
            )
            work_dir = root / "preprocessed/trial_02"
            with (
                mock.patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                mock.patch.object(subject.v141, "_RUNTIME", state),
                mock.patch.object(
                    subject.v141, "RECOVERY_LINEAGE", subject.BOUNDARY_LINEAGE
                ),
                mock.patch.object(
                    subject.v14,
                    "_verify_raw_identity",
                    side_effect=AssertionError("raw identity route forbidden"),
                ),
                mock.patch.object(
                    subject.v141.recovery_converter,
                    "convert_trial",
                    side_effect=AssertionError("converter route forbidden"),
                ),
                mock.patch.object(
                    subject.v141.recovery_converter,
                    "run_ik_from_setup",
                    side_effect=AssertionError("IK route forbidden"),
                ),
            ):
                artifacts = subject._prepare_trial02_from_parent(
                    protocol,
                    trial_id="02",
                    stage="development",
                    work_dir=work_dir,
                    access_receipt=None,
                )
                lock = subject.v141._validated_preprocessing_lock(
                    protocol, artifacts
                )
            self.assertEqual(
                subject._sha256(artifacts.ik_motion),
                "4018b3001a5d293edda799839158f4c154747af5c908a2eb530dae3e37e5a982",
            )
            self.assertEqual(
                lock.name, "treadmill_02_01_preprocessing_lock.json"
            )
            self.assertEqual(len(list(work_dir.iterdir())), 10)

    def test_configured_runtime_restores_every_global_after_exception(self) -> None:
        parent_names = (
            "DEFAULT_PROTOCOL",
            "DEFAULT_OUTPUT_DIR",
            "DEFAULT_EXECUTION_LEDGER",
            "prepare_trial",
            "_evaluate_trial",
            "evaluate_continuous_candidate",
            "root_safe_isolated",
            "_write_json_exclusive",
            "_plateau_references",
        )
        wrapper_names = (
            "DEFAULT_PROTOCOL",
            "DEFAULT_OUTPUT_DIR",
            "DEFAULT_EXECUTION_LEDGER",
            "PROTOCOL_ID",
            "RECOVERY_ID",
            "RECOVERY_LINEAGE",
        )
        parent_before = {name: getattr(subject.v14, name) for name in parent_names}
        wrapper_before = {name: getattr(subject.v141, name) for name in wrapper_names}
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError, "synthetic"):
                with subject._configured_runtime({}, Path(tmp)):
                    self.assertIs(
                        subject.v14._plateau_references,
                        subject._plateau_references_v142,
                    )
                    raise RuntimeError("synthetic")
        self.assertEqual(
            parent_before, {name: getattr(subject.v14, name) for name in parent_names}
        )
        self.assertEqual(
            wrapper_before,
            {name: getattr(subject.v141, name) for name in wrapper_names},
        )
        self.assertIsNone(subject.v141._RUNTIME)

    def test_alternate_output_is_rejected_without_writes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            alternate = root / "alternate"
            alternate.mkdir()
            sentinel = alternate / "run_start_receipt.json"
            sentinel.write_bytes(b"sentinel\n")
            before = sentinel.read_bytes()
            with (
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(["--execute", "--output-dir", str(alternate)])
            after = sentinel.read_bytes()
        self.assertEqual(code, 1)
        self.assertEqual(after, before)

    def test_consumed_retry_is_byte_immutable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            canonical.mkdir()
            receipt = canonical / "run_start_receipt.json"
            receipt.write_bytes(b"sentinel\n")
            ledger = root / "ledger.json"
            ledger.write_bytes(b"consumed\n")
            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(["--execute", "--output-dir", str(canonical)])
            receipt_after = receipt.read_bytes()
            failure_exists = (canonical / "failure.json").exists()
        self.assertEqual(code, 2)
        self.assertEqual(receipt_after, b"sentinel\n")
        self.assertFalse(failure_exists)

    def test_midrun_failure_is_persisted_once(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            canonical = root / "canonical"
            ledger = root / "ledger.json"

            def fail(*_args: object) -> dict[str, object]:
                canonical.mkdir()
                (canonical / "run_start_receipt.json").write_text(
                    "{}\n", encoding="utf-8"
                )
                ledger.write_text("{}\n", encoding="utf-8")
                raise RuntimeError("synthetic failure")

            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(subject, "DEFAULT_EXECUTION_LEDGER", ledger),
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                mock.patch.object(subject, "run_cross_speed_protocol", side_effect=fail),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(["--execute", "--output-dir", str(canonical)])
            failure = json.loads((canonical / "failure.json").read_text())
        self.assertEqual(code, 1)
        self.assertEqual(
            failure["status"],
            "ERROR_AFTER_V14_2_BOUNDARY_RECOVERY_DESTINATION_CONSUMED",
        )
        self.assertFalse(failure["rerun_allowed"])

    def test_completed_scientific_fail_returns_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            canonical = Path(tmp) / "canonical"
            with (
                mock.patch.object(subject, "DEFAULT_OUTPUT_DIR", canonical),
                mock.patch.object(
                    subject, "DEFAULT_EXECUTION_LEDGER", Path(tmp) / "ledger.json"
                ),
                mock.patch.object(subject, "_preflight_no_clobber"),
                mock.patch.object(subject, "load_and_validate_protocol", return_value={}),
                mock.patch.object(
                    subject, "run_cross_speed_protocol", return_value={"ok": False}
                ),
                redirect_stdout(io.StringIO()),
            ):
                code = subject.main(["--execute", "--output-dir", str(canonical)])
        self.assertEqual(code, 1)


if __name__ == "__main__":
    unittest.main()
