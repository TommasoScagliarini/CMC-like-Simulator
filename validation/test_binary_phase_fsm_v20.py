from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_ROOT))

from binary_phase_fsm import (  # noqa: E402
    BinaryPhaseFSM,
    BinaryPhaseFSMConfig,
)


def sample(time_s: float, heel: bool, toe: bool) -> dict:
    return {
        "time_s": time_s,
        "left_heel_contact": heel,
        "left_toe_contact": toe,
    }


class BinaryPhaseFSMV20Tests(unittest.TestCase):
    def new_fsm(self, heel: bool = False, toe: bool = False) -> BinaryPhaseFSM:
        fsm = BinaryPhaseFSM()
        fsm.reset(time_s=0.0, heel_contact=heel, toe_contact=toe)
        return fsm

    def test_frozen_timing_contract(self) -> None:
        cfg = BinaryPhaseFSMConfig()
        self.assertEqual(cfg.sample_dt_s, 0.001)
        self.assertEqual(cfg.debounce_s, 0.005)
        with self.assertRaises(ValueError):
            BinaryPhaseFSMConfig(sample_dt_s=0.003, debounce_s=0.005)
        with self.assertRaises(ValueError):
            BinaryPhaseFSMConfig(sample_dt_s=float("nan"))

    def test_reset_primes_all_four_words_without_event(self) -> None:
        expected = {
            (False, False): ("AIR", 0, False),
            (False, True): ("TOE", 1, True),
            (True, False): ("HEEL", 2, True),
            (True, True): ("BOTH", 3, True),
        }
        for bits, state in expected.items():
            with self.subTest(bits=bits):
                payload = self.new_fsm(*bits).payload()
                self.assertEqual(payload["raw_contact_state"], state[0])
                self.assertEqual(payload["raw_contact_state_id"], state[1])
                self.assertEqual(payload["in_contact"], state[2])
                self.assertEqual(payload["events_this_step"], [])

    def test_hs_leader_and_exact_elapsed_debounce(self) -> None:
        for heel, toe, leader in (
            (True, False, "heel"),
            (False, True, "toe"),
            (True, True, "both"),
        ):
            with self.subTest(leader=leader):
                fsm = self.new_fsm()
                for index in range(1, 6):
                    payload = fsm.update_sample(
                        time_s=index / 1000,
                        heel_contact=heel,
                        toe_contact=toe,
                    )
                    self.assertEqual(payload["events_this_step"], [])
                payload = fsm.update_sample(
                    time_s=0.006,
                    heel_contact=heel,
                    toe_contact=toe,
                )
                events = payload["events_this_step"]
                self.assertEqual(len(events), 1)
                event = events[0]
                self.assertEqual(event["event"], "heel_strike")
                self.assertAlmostEqual(event["event_time_s"], 0.001)
                self.assertAlmostEqual(event["confirmed_time_s"], 0.006)
                self.assertAlmostEqual(
                    event["confirmed_time_s"] - event["event_time_s"], 0.005
                )
                self.assertEqual(event["contact_leader"], leader)

    def test_four_milliseconds_is_cancelled_five_is_confirmed(self) -> None:
        fsm = self.new_fsm()
        for index in range(1, 6):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=True,
                toe_contact=False,
            )
            self.assertEqual(payload["events_this_step"], [])
        payload = fsm.update_sample(
            time_s=0.006,
            heel_contact=False,
            toe_contact=False,
        )
        self.assertIsNone(payload["pending_event"])
        self.assertEqual(payload["event_count"]["heel_strike"], 0)

        for index in range(7, 13):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=True,
                toe_contact=False,
            )
        self.assertEqual(payload["events_this_step"][0]["event"], "heel_strike")
        self.assertAlmostEqual(payload["events_this_step"][0]["event_time_s"], 0.007)

    def test_regional_changes_do_not_reset_or_duplicate_hs(self) -> None:
        fsm = self.new_fsm()
        words = [
            (True, False),
            (True, True),
            (False, True),
            (True, True),
            (True, False),
            (True, True),
        ]
        for index, bits in enumerate(words, start=1):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=bits[0],
                toe_contact=bits[1],
            )
        event = payload["events_this_step"][0]
        self.assertEqual(event["event"], "heel_strike")
        self.assertAlmostEqual(event["event_time_s"], 0.001)

        for index, bits in enumerate(
            [(True, False), (False, True), (True, True)], start=7
        ):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=bits[0],
                toe_contact=bits[1],
            )
            self.assertEqual(payload["events_this_step"], [])
        self.assertEqual(payload["event_count"]["heel_strike"], 1)

    def test_to_requires_stable_air_and_marks_partial_start(self) -> None:
        fsm = self.new_fsm(heel=True)
        for index in range(1, 5):
            fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=False,
            )
        payload = fsm.update_sample(
            time_s=0.005,
            heel_contact=True,
            toe_contact=False,
        )
        self.assertEqual(payload["event_count"]["toe_off"], 0)

        for index in range(6, 12):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=False,
            )
        event = payload["events_this_step"][0]
        self.assertEqual(event["event"], "toe_off")
        self.assertAlmostEqual(event["event_time_s"], 0.006)
        self.assertAlmostEqual(event["confirmed_time_s"], 0.011)
        self.assertTrue(event["startup_partial_stance"])

    def test_pending_candidate_crosses_policy_boundary(self) -> None:
        fsm = self.new_fsm()
        first = [sample(i / 1000, i >= 9, False) for i in range(1, 11)]
        payload = fsm.update_policy_step(
            time_s=0.010,
            previous_time_s=0.0,
            sensor_samples=first,
        )
        self.assertEqual(payload["events_this_step"], [])
        self.assertAlmostEqual(payload["pending_event"]["event_time_s"], 0.009)

        second = [sample(i / 1000, True, False) for i in range(11, 21)]
        payload = fsm.update_policy_step(
            time_s=0.020,
            previous_time_s=0.010,
            sensor_samples=second,
        )
        event = payload["events_this_step"][0]
        self.assertAlmostEqual(event["event_time_s"], 0.009)
        self.assertAlmostEqual(event["confirmed_time_s"], 0.014)
        self.assertAlmostEqual(event["delivered_time_s"], 0.020)

    def test_batch_failure_is_atomic_and_fail_closed(self) -> None:
        fsm = self.new_fsm()
        before = fsm.payload()
        malformed = [sample(i / 1000, False, False) for i in range(1, 11)]
        malformed[5]["left_heel_contact"] = 1
        with self.assertRaises(TypeError):
            fsm.update_policy_step(
                time_s=0.010,
                previous_time_s=0.0,
                sensor_samples=malformed,
            )
        self.assertEqual(fsm.payload(), before)

        duplicate = [sample(i / 1000, False, False) for i in range(1, 11)]
        duplicate[6]["time_s"] = duplicate[5]["time_s"]
        with self.assertRaises(ValueError):
            fsm.update_policy_step(
                time_s=0.010,
                previous_time_s=0.0,
                sensor_samples=duplicate,
            )
        self.assertEqual(fsm.payload(), before)

    def test_exact_fields_missing_samples_and_nonfinite_are_rejected(self) -> None:
        fsm = self.new_fsm()
        missing = [sample(i / 1000, False, False) for i in range(1, 10)]
        with self.assertRaises(ValueError):
            fsm.update_policy_step(
                time_s=0.010,
                previous_time_s=0.0,
                sensor_samples=missing,
            )
        with self.assertRaises(ValueError):
            fsm.update_sample(
                time_s=float("nan"), heel_contact=False, toe_contact=False
            )
        with self.assertRaises(TypeError):
            fsm.update_sample(
                time_s=0.001, heel_contact=1, toe_contact=False
            )

    def test_scalar_and_batch_have_exact_causal_parity(self) -> None:
        trace = [
            sample(i / 1000, 2 <= i <= 12, 4 <= i <= 8)
            for i in range(1, 21)
        ]
        batch_fsm = self.new_fsm()
        batch_events: list[dict] = []
        for start in (0, 10):
            payload = batch_fsm.update_policy_step(
                time_s=(start + 10) / 1000,
                previous_time_s=start / 1000,
                sensor_samples=trace[start : start + 10],
            )
            batch_events.extend(payload["events_this_step"])

        scalar_fsm = self.new_fsm()
        scalar_events: list[dict] = []
        for index, entry in enumerate(trace, start=1):
            delivery = 0.010 if index <= 10 else 0.020
            payload = scalar_fsm.update_sample(
                time_s=entry["time_s"],
                heel_contact=entry["left_heel_contact"],
                toe_contact=entry["left_toe_contact"],
                delivered_time_s=delivery,
            )
            scalar_events.extend(payload["events_this_step"])

        self.assertEqual(scalar_events, batch_events)
        scalar_state = scalar_fsm.payload()
        batch_state = batch_fsm.payload()
        for transient in (
            "events_this_step",
            "contact_state_transitions_this_step",
        ):
            scalar_state.pop(transient)
            batch_state.pop(transient)
        self.assertEqual(scalar_state, batch_state)

    def test_payload_is_strict_json(self) -> None:
        fsm = self.new_fsm()
        fsm.update_sample(
            time_s=0.001,
            heel_contact=True,
            toe_contact=False,
        )
        json.dumps(fsm.payload(), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
