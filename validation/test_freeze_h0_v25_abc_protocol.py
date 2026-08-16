"""Read-only/temp-destination tests for the non-executable H0/V25 protocol."""

from __future__ import annotations

import builtins
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
VALIDATION_ROOT = REPO_ROOT / "validation"
if str(VALIDATION_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(VALIDATION_ROOT))

import freeze_h0_v25_abc_protocol as subject  # noqa: E402


class H0V25ABCProtocolFreezeTests(unittest.TestCase):
    def temporary_dir(self) -> Path:
        temporary = tempfile.TemporaryDirectory(dir=VALIDATION_ROOT)
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def protocol(self) -> dict:
        temporary = self.temporary_dir()
        with patch.object(subject, "DESTINATION", temporary / "protocol.json"):
            return subject.build_protocol_payload()

    def test_preflight_is_read_only_and_pins_every_hash_and_tree(self) -> None:
        temporary = self.temporary_dir()
        destination = temporary / "protocol.json"
        before = {
            label: (path.read_bytes(), subject.shadow.sha256_file(path))
            for label, (path, _expected) in subject.PINNED.items()
        }
        real_import = builtins.__import__

        def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
            forbidden = ("opensim", "ray", "torch", "pickle")
            if name == forbidden or any(
                name == prefix or name.startswith(prefix + ".")
                for prefix in forbidden
            ):
                raise AssertionError(f"protocol preflight imported runtime module {name}")
            return real_import(name, globals, locals, fromlist, level)

        with patch.object(subject, "DESTINATION", destination), patch(
            "builtins.__import__", side_effect=guarded_import
        ):
            preflight = subject.preflight_unfrozen()

        self.assertFalse(destination.exists())
        self.assertEqual(
            preflight["status"],
            "H0_V25_ABC_PROTOCOL_LOCK_READY_UNWRITTEN",
        )
        self.assertFalse(preflight["execution_authorized"])
        self.assertTrue(all(preflight["protocol_payload"]["assertions"].values()))
        self.assertEqual(
            preflight["lock_record_if_frozen"],
            subject.shadow.payload_record(destination, preflight["protocol_payload"]),
        )
        for label, (path, expected_sha256) in subject.PINNED.items():
            with self.subTest(label=label):
                self.assertEqual(subject.shadow.sha256_file(path), expected_sha256)
                self.assertEqual(before[label], (path.read_bytes(), expected_sha256))

        checkpoint = preflight["protocol_payload"]["h0_identity"]["checkpoint_tree"]
        module = preflight["protocol_payload"]["h0_identity"]["rl_module_tree"]
        for key, value in subject.EXPECTED_CHECKPOINT_TREE.items():
            self.assertEqual(checkpoint[key], value)
        for key, value in subject.EXPECTED_RL_MODULE_TREE.items():
            self.assertEqual(module[key], value)

    def test_matrix_has_exactly_twelve_units_and_eighteen_real_rollouts(self) -> None:
        matrix = self.protocol()["matrix"]
        self.assertEqual(matrix["condition_count"], 6)
        self.assertEqual(matrix["paired_ab_unit_count"], 6)
        self.assertEqual(matrix["c_unit_count"], 6)
        self.assertEqual(matrix["protocol_unit_count"], 12)
        self.assertEqual(matrix["underlying_rollout_count"], 18)
        self.assertEqual(len(matrix["units"]), 12)
        self.assertEqual(len(matrix["rollouts"]), 18)
        self.assertEqual(len({row["unit_id"] for row in matrix["units"]}), 12)
        self.assertEqual(
            len({row["rollout_id"] for row in matrix["rollouts"]}),
            18,
        )

        paired = [row for row in matrix["units"] if row["unit_type"] == "indivisible_paired_ab"]
        active = [row for row in matrix["units"] if row["unit_type"] == "single_active_c"]
        self.assertEqual(len(paired), 6)
        self.assertTrue(all(len(row["rollout_ids"]) == 2 for row in paired))
        self.assertEqual(len(active), 6)
        self.assertTrue(all(len(row["rollout_ids"]) == 1 for row in active))
        self.assertTrue(
            all(row["prerequisite"] == "ALL_SIX_PAIRED_AB_UNITS_PASS" for row in active)
        )

    def test_a_and_b_share_every_input_except_shadow_fsm_execution(self) -> None:
        matrix = self.protocol()["matrix"]
        case_a = dict(matrix["cases"]["A"])
        case_b = dict(matrix["cases"]["B"])
        self.assertEqual(case_a.pop("case_name"), "legacy_control")
        self.assertEqual(case_b.pop("case_name"), "binary_shadow")
        self.assertEqual(case_a.pop("binary_phase_fsm_mode"), "disabled")
        self.assertEqual(case_b.pop("binary_phase_fsm_mode"), "binary_shadow")
        self.assertNotIn("scientific_bundle_contract_id", case_a)
        self.assertEqual(
            case_b.pop("scientific_bundle_contract_id"),
            subject.shadow.SCIENTIFIC_SHADOW_BUNDLE_ID,
        )
        self.assertEqual(case_a, case_b)
        self.assertEqual(
            case_a["binary_phase_event_contract_id"],
            subject.shadow.RUNTIME_V25_CONTRACT_ID,
        )
        self.assertEqual(case_a["binary_phase_debounce_s"], 0.005)

        case_c = matrix["cases"]["C"]
        self.assertEqual(
            case_c["binary_phase_fsm_mode"],
            "binary_active_FUTURE_UNAVAILABLE",
        )
        self.assertFalse(case_c["left_fallback_allowed"])
        self.assertEqual(
            case_c["binary_phase_event_contract_id"],
            subject.ACTIVE_RUNTIME_CONTRACT_ID,
        )
        self.assertTrue(case_c["v20_to_prosthetic_phase_fsm_adapter_required"])
        self.assertFalse(case_c["direct_phase_fsm_payload_replacement_allowed"])

    def test_gate_schema_freezes_projection_tape_events_and_sea_metrics(self) -> None:
        gates = self.protocol()["gates"]
        common = gates["common_per_rollout"]
        self.assertEqual(common["steps_exact"], 500)
        self.assertEqual(common["minimum_complete_valid_cycles"], 2)
        self.assertEqual(common["max_penetration_m_strictly_less_than"], 0.025)
        self.assertEqual(
            common["legacy_invalid_event_count"],
            "A_AND_B_PRESENT_FINITE_BIT_EXACT_NOT_REQUIRED_ZERO",
        )

        paired = gates["paired_ab"]
        self.assertEqual(
            paired["top_level_diff_allowlist"],
            {
                "rule": "key_name_startswith_exact",
                "prefix": "binary_phase_",
                "other_exclusions": [],
            },
        )
        self.assertEqual(paired["raw_v25_input_journal"]["one_ms_sample_count"], 5000)
        self.assertTrue(gates["action_tape"]["simple_reseed_for_B_forbidden"])
        self.assertEqual(gates["action_tape"]["expected_sigma"], 0.005)
        self.assertTrue(gates["action_tape"]["C_is_closed_loop_and_never_replays_A_actions"])

        c_events = gates["c_events"]
        self.assertEqual(c_events["valid_words"]["AIR"], [0, 0])
        self.assertFalse(c_events["candidate_cancellation_before_debounce_is_invalid"])
        self.assertTrue(c_events["startup_partial_stance"]["leading_to_allowed"])
        self.assertEqual(c_events["binary_hard_invalid_unknown_unaccepted_count"], 0)

        sea = gates["c_condition_matched_nonregression"]["sea_metrics"]
        self.assertEqual(len(sea), 14)
        self.assertEqual({row["joint"] for row in sea}, {"knee", "ankle"})
        self.assertEqual(
            sum(row["signal"] == "tau_input_saturated" for row in sea),
            2,
        )

    def test_authority_data_and_terminal_closures_are_exact(self) -> None:
        protocol = self.protocol()
        self.assertEqual(
            protocol["status"],
            "H0_V25_ABC_PROTOCOL_FROZEN_EXECUTION_NOT_AUTHORIZED",
        )
        self.assertFalse(protocol["protocol_executed"])
        self.assertEqual(
            protocol["scientific_result"],
            "UNAVAILABLE_PROTOCOL_NOT_EXECUTED",
        )
        self.assertEqual(
            protocol["contracts"]["runtime_active_component_contract_id"],
            subject.ACTIVE_RUNTIME_CONTRACT_ID,
        )
        for key, value in protocol["authority"].items():
            self.assertIs(value, False, key)
        for key, value in protocol["unmet_execution_prerequisites"].items():
            self.assertIs(value, False, key)
        self.assertIn(
            "A_raw_v25_journal_capture_implemented_and_tested",
            protocol["unmet_execution_prerequisites"],
        )
        governance = protocol["data_governance"]
        self.assertEqual(governance["protected_trials_opened"], [])
        self.assertEqual(governance["protected_trials_closed"], ["05", "06"])
        self.assertEqual(governance["reserve_trials_opened"], [])
        self.assertEqual(governance["reserve_trials_closed"], ["03", "07"])
        terminal = protocol["stop_and_terminal_semantics"]
        self.assertEqual(terminal["A_failure"], "ERROR_H0_REFERENCE")
        self.assertEqual(terminal["AB_mismatch"], "ERROR_SHADOW_NONINTERFERENCE")
        self.assertEqual(
            terminal["C_scientific_failure_after_all_AB_pass"],
            "FAIL_H0_V25_COMPATIBILITY",
        )
        self.assertEqual(
            terminal["all_C_pass_after_all_AB_pass"],
            "PASS_H0_V25_COMPATIBLE",
        )
        refusal = protocol["historical_preflight_refusals_expected"]
        self.assertFalse(refusal["v24_or_v25_live_preflight_rerunnable"])
        self.assertTrue(refusal["historical_evidence_and_locks_immutable"])
        self.assertFalse(refusal["historical_script_or_receipt_rewrite_allowed"])

    def test_digest_namespaces_are_distinct_and_not_compared_as_strings(self) -> None:
        digests = self.protocol()["h0_identity"]["digests"]
        raw = digests["raw_module_state_file_sha256"]
        warm = digests["warm_start_actor_state_digest"]
        compare = digests["compare_policy_actor_digest"]
        self.assertEqual(raw, subject.PINNED["h0_module_state"][1])
        self.assertNotEqual(warm["value"], compare["value"])
        self.assertNotEqual(warm["algorithm_source"], compare["algorithm_source"])
        self.assertTrue(
            digests["historical_actor_digest_values_expected_to_differ_by_algorithm"]
        )

    def test_strict_json_and_exclusive_protocol_writer_fail_closed(self) -> None:
        temporary = self.temporary_dir()
        real_destination_state_before = os.path.lexists(subject.DESTINATION)
        duplicate = temporary / "duplicate.json"
        duplicate.write_text('{"x": 1, "x": 2}\n', encoding="utf-8")
        with self.assertRaises(subject.shadow.V25ShadowReadinessError):
            subject.shadow.strict_json_load(duplicate)
        nonfinite = temporary / "nonfinite.json"
        nonfinite.write_text('{"x": 1e999}\n', encoding="utf-8")
        with self.assertRaises(subject.shadow.V25ShadowReadinessError):
            subject.shadow.strict_json_load(nonfinite)

        destination = temporary / "protocol.json"
        with patch.object(subject, "DESTINATION", destination):
            protocol = subject.freeze_protocol()
            with self.assertRaisesRegex(subject.H0V25ProtocolFreezeError, "clobber"):
                subject.freeze_protocol()
        self.assertEqual(subject.shadow.strict_json_load(destination), protocol)
        self.assertEqual(
            os.path.lexists(subject.DESTINATION),
            real_destination_state_before,
        )


if __name__ == "__main__":
    unittest.main()
