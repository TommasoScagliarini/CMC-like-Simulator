"""Synthetic fail-closed tests for the prescribed two-sensor phase gate."""

from __future__ import annotations

import unittest

import numpy as np

from validation.validate_two_sensor_prescribed_replay import (
    PHASE_STANCE,
    PHASE_SWING,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    _confirmed_state_hold_gate,
    _phase_classification_gate,
)


class PrescribedPhaseGateTest(unittest.TestCase):
    def _fixture(self) -> tuple[
        np.ndarray,
        np.ndarray,
        dict[str, np.ndarray],
        dict[str, np.ndarray],
        dict[str, object],
    ]:
        times = np.round(np.arange(0.0, 2.01, 0.01), decimals=8)
        # One complete reference cycle: HS 0.50, TO 1.20, HS 1.70.
        prescribed_vertical_n = np.zeros(times.shape, dtype=float)
        prescribed_vertical_n[
            ((times >= 0.50) & (times < 1.20)) | (times >= 1.70)
        ] = 100.0
        reference_events = {
            "heel_strike": np.asarray([0.50, 1.70]),
            "toe_off": np.asarray([1.20]),
        }
        predicted_events = {
            "heel_strike": np.asarray([0.50, 1.70]),
            "toe_off": np.asarray([1.20]),
        }
        accepted = [
            {
                "event": "heel_strike",
                "event_time_s": 0.50,
                "confirmed_time_s": 0.53,
                "segment_valid": 1.0,
            },
            {
                "event": "toe_off",
                "event_time_s": 1.20,
                "confirmed_time_s": 1.23,
                "segment_valid": 1.0,
            },
            {
                "event": "heel_strike",
                "event_time_s": 1.70,
                "confirmed_time_s": 1.73,
                "segment_valid": 1.0,
            },
        ]
        state_id = np.zeros(times.shape, dtype=float)
        state_id[(times >= 0.53) & (times < 1.23)] = STANCE_AFTER_HS
        state_id[(times >= 1.23) & (times < 1.73)] = SWING_AFTER_TO
        state_id[times >= 1.73] = STANCE_AFTER_HS
        replay: dict[str, object] = {
            "state_id": state_id,
            "accepted": accepted,
        }
        return (
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            replay,
        )

    def _run_gate(self, replay: dict[str, object]) -> dict[str, object]:
        (
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            _original_replay,
        ) = self._fixture()
        return _phase_classification_gate(
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            replay,
            prescribed_threshold_n=20.0,
            hs_tolerance_s=0.050,
            to_tolerance_s=0.080,
            sensor_dwell_s=0.030,
        )

    def test_aligned_phase_sequence_passes(self) -> None:
        *_, replay = self._fixture()
        result = self._run_gate(replay)
        self.assertTrue(result["ok"])
        self.assertEqual(result["forbidden_mismatch_count"], 0)
        self.assertTrue(result["confirmed_state_hold_gate"]["ok"])

    def test_confirmation_delay_mismatch_only_in_window_passes(self) -> None:
        *_, replay = self._fixture()
        result = self._run_gate(replay)
        raw = result["strict_interval_raw_agreement"]
        settled = result["settled_outside_transition_windows_agreement"]
        self.assertGreater(raw["mismatches"], 0)
        self.assertEqual(settled["mismatches"], 0)
        self.assertTrue(result["ok"])

    def test_mismatch_far_from_events_fails(self) -> None:
        times, *_rest, replay = self._fixture()
        state_id = np.asarray(replay["state_id"]).copy()
        state_id[np.isclose(times, 1.40)] = STANCE_AFTER_HS
        replay["state_id"] = state_id
        result = self._run_gate(replay)
        self.assertFalse(result["ok"])
        self.assertEqual(result["forbidden_mismatch_times_s"], [1.4])

    def test_confirmed_state_not_held_until_next_transition_fails(self) -> None:
        times, *_rest, replay = self._fixture()
        predicted_phase = np.full(times.shape, PHASE_SWING, dtype=int)
        predicted_phase[(times >= 0.53) & (times < 0.80)] = PHASE_STANCE
        predicted_phase[times >= 1.73] = PHASE_STANCE
        result = _confirmed_state_hold_gate(
            times,
            predicted_phase,
            replay["accepted"],
        )
        self.assertFalse(result["ok"])
        self.assertGreater(result["records"][0]["mismatch_count"], 0)

    def test_validated_event_intervals_ignores_unaccepted_grf_spike(self) -> None:
        (
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            replay,
        ) = self._fixture()
        spike_index = int(np.flatnonzero(np.isclose(times, 1.40))[0])
        prescribed_vertical_n[spike_index] = 100.0

        validated = _phase_classification_gate(
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            replay,
            prescribed_threshold_n=20.0,
            hs_tolerance_s=0.050,
            to_tolerance_s=0.080,
            sensor_dwell_s=0.030,
            reference_phase_mode="validated_event_intervals",
        )
        instantaneous = _phase_classification_gate(
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_events,
            replay,
            prescribed_threshold_n=20.0,
            hs_tolerance_s=0.050,
            to_tolerance_s=0.080,
            sensor_dwell_s=0.030,
            reference_phase_mode="instantaneous_grf",
        )

        self.assertEqual(
            int(validated["_arrays"]["reference_phase"][spike_index]),
            PHASE_SWING,
        )
        self.assertTrue(validated["ok"])
        self.assertEqual(validated["forbidden_mismatch_count"], 0)
        self.assertEqual(
            int(instantaneous["_arrays"]["reference_phase"][spike_index]),
            PHASE_STANCE,
        )
        self.assertFalse(instantaneous["ok"])
        self.assertEqual(instantaneous["forbidden_mismatch_times_s"], [1.4])

    def test_confirmed_primary_window_does_not_add_sensor_dwell_twice(self) -> None:
        (
            times,
            prescribed_vertical_n,
            reference_events,
            _predicted_onsets,
            replay,
        ) = self._fixture()
        predicted_confirmations = {
            "heel_strike": np.asarray([0.53, 1.73]),
            "toe_off": np.asarray([1.23]),
        }
        result = _phase_classification_gate(
            times,
            prescribed_vertical_n,
            reference_events,
            predicted_confirmations,
            replay,
            prescribed_threshold_n=20.0,
            hs_tolerance_s=0.050,
            to_tolerance_s=0.080,
            sensor_dwell_s=0.030,
            reference_phase_mode="validated_event_intervals",
            primary_event_time_field="confirmed_time_s",
        )

        policy = result["mismatch_policy"]
        self.assertTrue(result["ok"])
        self.assertEqual(policy["primary_event_time_field"], "confirmed_time_s")
        self.assertAlmostEqual(
            policy["additional_dwell_applied_to_window_s"],
            0.0,
        )
        windows = {
            (item["event"], item["event_index"]): item
            for item in policy["transition_windows"]
        }
        self.assertAlmostEqual(
            windows[("heel_strike", 0)]["allowed_mismatch_window_end_s"],
            0.55,
        )
        self.assertAlmostEqual(
            windows[("toe_off", 0)]["allowed_mismatch_window_end_s"],
            1.28,
        )
        outside_window = result["_arrays"]["allowed_transition_mask"][
            np.isclose(times, 0.56)
        ]
        self.assertFalse(bool(outside_window[0]))

    def test_legacy_onset_primary_window_retains_sensor_dwell(self) -> None:
        *_, replay = self._fixture()
        result = self._run_gate(replay)

        policy = result["mismatch_policy"]
        self.assertTrue(result["ok"])
        self.assertEqual(policy["primary_event_time_field"], "event_time_s")
        self.assertAlmostEqual(
            policy["additional_dwell_applied_to_window_s"],
            0.030,
        )
        windows = {
            (item["event"], item["event_index"]): item
            for item in policy["transition_windows"]
        }
        self.assertAlmostEqual(
            windows[("heel_strike", 0)]["allowed_mismatch_window_end_s"],
            0.58,
        )
        self.assertAlmostEqual(
            windows[("toe_off", 0)]["allowed_mismatch_window_end_s"],
            1.31,
        )


if __name__ == "__main__":
    unittest.main()
