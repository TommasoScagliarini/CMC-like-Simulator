"""Synthetic-only tests for the V14.1 warm-up certificate."""

from __future__ import annotations

import copy
import unittest
from typing import Any

from validation import two_sensor_v14_1_warmup as subject


class SyntheticProtocolError(RuntimeError):
    pass


def _valid_inputs() -> dict[str, Any]:
    times = [index * 0.1 for index in range(21)]
    states = [subject.WAIT_HS] * len(times)
    for index, time_s in enumerate(times):
        if 0.2 <= time_s < 0.5:
            states[index] = subject.STANCE_AFTER_HS
        elif 0.5 <= time_s < 0.8:
            states[index] = subject.SWING_AFTER_TO
        elif time_s >= 0.8:
            states[index] = subject.STANCE_AFTER_HS
    return {
        "analysis_start_s": 0.0,
        "first_scoreable_hs_s": 1.09,
        "sample_dt_s": 0.1,
        "sample_times_s": times,
        "state_ids": states,
        "detector_accepted_events": [
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
            {
                "event": "heel_strike",
                "event_time_s": 0.78,
                "confirmed_time_s": 0.80,
            },
        ],
        "detector_invalid_steps": [],
        "prescribed_events": {
            "heel_strike": [0.05, 0.75, 1.09],
            "toe_off": [0.30, 0.90],
        },
    }


def _passing_certificate() -> dict[str, Any]:
    return subject.certify_warmup(**_valid_inputs())


def _failing_certificate() -> dict[str, Any]:
    inputs = _valid_inputs()
    inputs["detector_accepted_events"] = inputs["detector_accepted_events"][:2]
    return subject.certify_warmup(**inputs)


class WarmupCertificationTests(unittest.TestCase):
    def test_passes_with_detector_and_prescribed_full_cycles(self) -> None:
        certificate = _passing_certificate()

        self.assertTrue(certificate["ok"])
        self.assertEqual(certificate["status"], "PASS_WARMUP_CERTIFIED")
        self.assertAlmostEqual(certificate["cutoff_s"], 1.0)
        self.assertEqual(certificate["detector"]["complete_cycle_count"], 1)
        self.assertEqual(certificate["prescribed"]["complete_cycle_count"], 1)
        self.assertEqual(
            certificate["detector"]["cycle_time_field"], "confirmed_time_s"
        )
        self.assertEqual(
            certificate["cutoff_state"]["state_name"], "STANCE_AFTER_HS"
        )

    def test_detector_requires_complete_confirmed_cycle_before_cutoff(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"][-1]["event_time_s"] = 0.95
        inputs["detector_accepted_events"][-1]["confirmed_time_s"] = 1.02

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertEqual(certificate["detector"]["complete_cycle_count"], 0)

    def test_prescribed_requires_complete_cycle_before_cutoff(self) -> None:
        inputs = _valid_inputs()
        inputs["prescribed_events"] = {
            "heel_strike": [0.75, 1.09],
            "toe_off": [0.90],
        }

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertEqual(certificate["prescribed"]["complete_cycle_count"], 0)

    def test_timeout_event_in_warmup_fails(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"].insert(
            2, {"event": "timeout", "observed_at_s": 0.60}
        )

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertFalse(certificate["checks"]["zero_timeout_transitions"])

    def test_invalid_step_in_warmup_fails(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_invalid_steps"] = [{"observed_at_s": 0.60}]

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertFalse(certificate["checks"]["zero_invalid_transitions"])

    def test_failure_state_at_cutoff_fails(self) -> None:
        inputs = _valid_inputs()
        inputs["state_ids"][10] = subject.TIMEOUT

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertFalse(certificate["checks"]["zero_timeout_transitions"])
        self.assertFalse(certificate["checks"]["cutoff_state_non_failure"])

    def test_unknown_state_at_cutoff_fails_without_becoming_known(self) -> None:
        inputs = _valid_inputs()
        inputs["state_ids"][10] = 99

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertFalse(certificate["checks"]["cutoff_state_known"])
        self.assertEqual(certificate["cutoff_state"]["state_name"], "UNKNOWN")

    def test_unknown_state_inside_warmup_fails_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["state_ids"][3] = 99

        certificate = subject.certify_warmup(**inputs)

        self.assertFalse(certificate["ok"])
        self.assertFalse(certificate["checks"]["zero_invalid_transitions"])
        self.assertEqual(certificate["invalid"]["unknown_state_sample_count"], 1)

    def test_latest_nonfuture_state_may_be_one_cadence_before_cutoff(self) -> None:
        inputs = _valid_inputs()
        inputs["first_scoreable_hs_s"] = 1.085
        inputs["prescribed_events"]["heel_strike"][-1] = 1.085

        certificate = subject.certify_warmup(**inputs)

        self.assertTrue(certificate["ok"])
        self.assertAlmostEqual(certificate["cutoff_s"], 0.995)
        self.assertAlmostEqual(certificate["cutoff_state"]["sample_time_s"], 0.9)
        self.assertAlmostEqual(certificate["cutoff_state"]["sample_lag_s"], 0.095)

    def test_malformed_sample_grid_fails_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["sample_times_s"][5] = 0.55

        with self.assertRaisesRegex(
            subject.WarmupValidationError, "sample grid is inconsistent"
        ):
            subject.certify_warmup(**inputs)

    def test_noninteger_state_id_fails_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["state_ids"][3] = 1.5

        with self.assertRaisesRegex(subject.WarmupValidationError, "integer-valued"):
            subject.certify_warmup(**inputs)

    def test_malformed_prescribed_order_fails_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["prescribed_events"]["toe_off"][0] = 0.80

        with self.assertRaisesRegex(subject.WarmupValidationError, "alternate"):
            subject.certify_warmup(**inputs)

    def test_first_scoreable_hs_must_be_prescribed(self) -> None:
        inputs = _valid_inputs()
        inputs["first_scoreable_hs_s"] = 1.08

        with self.assertRaisesRegex(subject.WarmupValidationError, "absent"):
            subject.certify_warmup(**inputs)

    def test_invalid_record_without_time_fails_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_invalid_steps"] = [{}]

        with self.assertRaisesRegex(subject.WarmupValidationError, "no event timestamp"):
            subject.certify_warmup(**inputs)

    def test_detector_sequence_must_be_hs_to_toe_off_to_hs(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"][1]["event"] = "heel_strike"

        with self.assertRaisesRegex(subject.WarmupValidationError, "not HS--TO--HS"):
            subject.certify_warmup(**inputs)

    def test_simultaneous_detector_events_fail_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"][1]["event_time_s"] = 0.18
        inputs["detector_accepted_events"][1]["confirmed_time_s"] = 0.20

        with self.assertRaisesRegex(subject.WarmupValidationError, "strictly"):
            subject.certify_warmup(**inputs)

    def test_inverted_detector_onsets_fail_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"][1]["event_time_s"] = 0.17

        with self.assertRaisesRegex(subject.WarmupValidationError, "onsets"):
            subject.certify_warmup(**inputs)

    def test_inconsistent_timeout_timestamps_fail_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_accepted_events"].insert(
            2,
            {
                "event": "timeout",
                "observed_at_s": 0.60,
                "confirmed_time_s": 1.20,
            },
        )

        with self.assertRaisesRegex(subject.WarmupValidationError, "inconsistent"):
            subject.certify_warmup(**inputs)

    def test_inconsistent_invalid_timestamps_fail_closed(self) -> None:
        inputs = _valid_inputs()
        inputs["detector_invalid_steps"] = [
            {"observed_at_s": 0.60, "confirmed_time_s": 1.20}
        ]

        with self.assertRaisesRegex(subject.WarmupValidationError, "inconsistent"):
            subject.certify_warmup(**inputs)


class CertificateApplicationTests(unittest.TestCase):
    def test_passing_v13_certificate_preserves_existing_full_gate(self) -> None:
        row = {"candidate_id": "V13_BASELINE", "unit_full_gate_ok": True}

        applied = subject.apply_warmup_certificate(
            row, _passing_certificate(), is_v13=True
        )

        self.assertTrue(applied["warmup_ok"])
        self.assertTrue(applied["unit_full_gate_ok"])
        self.assertNotIn("warmup_ok", row)

    def test_failing_v13_raises_supplied_protocol_error(self) -> None:
        with self.assertRaisesRegex(SyntheticProtocolError, "V13 warm-up"):
            subject.apply_warmup_certificate(
                {"candidate_id": "V13_BASELINE", "unit_full_gate_ok": True},
                _failing_certificate(),
                is_v13=True,
                protocol_error_type=SyntheticProtocolError,
            )

    def test_failing_v13_defaults_to_runtime_error(self) -> None:
        with self.assertRaises(RuntimeError):
            subject.apply_warmup_certificate(
                {"candidate_id": "V13_BASELINE"},
                _failing_certificate(),
                is_v13=True,
            )

    def test_failing_nonbaseline_forces_both_flags_false(self) -> None:
        row = {"candidate_id": "challenger", "unit_full_gate_ok": True}

        applied = subject.apply_warmup_certificate(
            row, _failing_certificate(), is_v13=False
        )

        self.assertFalse(applied["warmup_ok"])
        self.assertFalse(applied["unit_full_gate_ok"])
        self.assertTrue(row["unit_full_gate_ok"])

    def test_passing_warmup_never_upgrades_a_failed_full_gate(self) -> None:
        applied = subject.apply_warmup_certificate(
            {"candidate_id": "challenger", "unit_full_gate_ok": False},
            _passing_certificate(),
            is_v13=False,
        )

        self.assertTrue(applied["warmup_ok"])
        self.assertFalse(applied["unit_full_gate_ok"])

    def test_tampered_certificate_fails_closed(self) -> None:
        certificate = _passing_certificate()
        certificate["checks"][
            "detector_complete_hs_to_toe_off_to_hs_cycle"
        ] = False

        with self.assertRaisesRegex(subject.WarmupValidationError, "inconsistent"):
            subject.apply_warmup_certificate(
                {"candidate_id": "challenger"},
                certificate,
                is_v13=False,
            )

    def test_tampered_cutoff_state_fails_closed(self) -> None:
        certificate = _passing_certificate()
        certificate["cutoff_state"]["state_id"] = subject.TIMEOUT

        with self.assertRaisesRegex(subject.WarmupValidationError, "inconsistent"):
            subject.apply_warmup_certificate(
                {"candidate_id": "challenger"},
                certificate,
                is_v13=False,
            )

    def test_tampered_timeout_evidence_fails_closed(self) -> None:
        certificate = _passing_certificate()
        certificate["detector"]["timeout_transition_times_s"] = [0.5]

        with self.assertRaisesRegex(subject.WarmupValidationError, "inconsistent"):
            subject.apply_warmup_certificate(
                {"candidate_id": "challenger"},
                certificate,
                is_v13=False,
            )

    def test_fabricated_detector_cycle_fails_closed(self) -> None:
        certificate = _failing_certificate()
        certificate["detector"]["complete_cycles"] = [{}]
        certificate["detector"]["complete_cycle_count"] = 1
        certificate["checks"][
            "detector_complete_hs_to_toe_off_to_hs_cycle"
        ] = True
        certificate["ok"] = True
        certificate["status"] = "PASS_WARMUP_CERTIFIED"

        with self.assertRaisesRegex(subject.WarmupValidationError, "schema"):
            subject.apply_warmup_certificate(
                {"candidate_id": "challenger"},
                certificate,
                is_v13=False,
            )

    def test_out_of_bounds_cycle_fails_closed(self) -> None:
        certificate = _passing_certificate()
        certificate["detector"]["complete_cycles"][0][
            "closing_hs_confirmed_s"
        ] = certificate["cutoff_s"] + 0.01

        with self.assertRaisesRegex(subject.WarmupValidationError, "order/bounds"):
            subject.apply_warmup_certificate(
                {"candidate_id": "challenger"},
                certificate,
                is_v13=False,
            )


class RootSafetyWarmupTests(unittest.TestCase):
    def test_all_units_must_explicitly_pass_warmup(self) -> None:
        rows = [
            {"v14_stage_unit": "u1", "warmup_ok": True},
            {"v14_stage_unit": "u2", "warmup_ok": True},
        ]

        result = subject.apply_root_safe_warmup({"ok": True}, rows)

        self.assertTrue(result["ok"])
        self.assertTrue(result["warmup"]["ok"])

    def test_false_warmup_is_rejected_by_root_safety(self) -> None:
        rows = [
            {"v14_stage_unit": "u1", "warmup_ok": True},
            {"v14_stage_unit": "u2", "warmup_ok": False},
        ]

        result = subject.apply_root_safe_warmup({"ok": True}, rows)

        self.assertFalse(result["ok"])
        self.assertEqual(result["warmup"]["failed_or_missing_units"], ["u2"])

    def test_missing_or_empty_warmup_is_fail_closed(self) -> None:
        self.assertFalse(subject.root_safe_warmup_ok([]))
        self.assertFalse(
            subject.root_safe_warmup_ok([{"v14_stage_unit": "u1"}])
        )

    def test_original_root_failure_cannot_be_upgraded(self) -> None:
        result = subject.apply_root_safe_warmup(
            {"ok": False}, [{"v14_stage_unit": "u1", "warmup_ok": True}]
        )

        self.assertFalse(result["ok"])

    def test_duplicate_unit_ids_fail_closed(self) -> None:
        rows = [
            {"v14_stage_unit": "u1", "warmup_ok": True},
            {"v14_stage_unit": "u1", "warmup_ok": True},
        ]

        with self.assertRaisesRegex(subject.WarmupValidationError, "duplicated"):
            subject.apply_root_safe_warmup({"ok": True}, rows)

    def test_root_safety_input_is_not_mutated(self) -> None:
        original = {"ok": True, "units": {"u1": {"ok": True}}}
        before = copy.deepcopy(original)

        subject.apply_root_safe_warmup(
            original, [{"v14_stage_unit": "u1", "warmup_ok": True}]
        )

        self.assertEqual(original, before)


if __name__ == "__main__":
    unittest.main()
