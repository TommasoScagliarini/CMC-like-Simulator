from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_ROOT))

from binary_phase_fsm_v26 import (  # noqa: E402
    HeelQualifiedBinaryPhaseFSM,
    HeelQualifiedBinaryPhaseFSMConfig,
    V26_EVENT_CONTRACT_ID,
    V26_SOURCE,
)


def sample(time_s: float, heel: bool, toe: bool) -> dict:
    return {
        "time_s": time_s,
        "left_heel_contact": heel,
        "left_toe_contact": toe,
    }


class HeelQualifiedBinaryPhaseFSMV26Tests(unittest.TestCase):
    def new_fsm(
        self,
        heel: bool = False,
        toe: bool = False,
    ) -> HeelQualifiedBinaryPhaseFSM:
        fsm = HeelQualifiedBinaryPhaseFSM()
        fsm.reset(time_s=0.0, heel_contact=heel, toe_contact=toe)
        return fsm

    def test_frozen_contract(self) -> None:
        cfg = HeelQualifiedBinaryPhaseFSMConfig()
        self.assertEqual(cfg.source, V26_SOURCE)
        self.assertEqual(cfg.event_contract_id, V26_EVENT_CONTRACT_ID)
        self.assertEqual(cfg.sample_dt_s, 0.001)
        self.assertEqual(cfg.debounce_s, 0.005)
        self.assertEqual(cfg.max_delivery_delay_s, 0.010)

    def test_toe_only_during_swing_is_diagnostic_without_hs(self) -> None:
        fsm = self.new_fsm()
        for index in range(1, 101):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=True,
            )
            self.assertEqual(payload["events_this_step"], [])
            self.assertFalse(payload["in_contact"])
        self.assertEqual(payload["raw_contact_state"], "TOE")
        self.assertEqual(payload["stable_contact_state"], "TOE")
        self.assertEqual(payload["event_count"]["heel_strike"], 0)
        self.assertEqual(payload["gait_phase"], "SWING")

    def test_heel_qualifies_hs_after_toe_only_episode(self) -> None:
        fsm = self.new_fsm()
        for index in range(1, 21):
            fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=True,
            )
        for index in range(21, 27):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=True,
                toe_contact=True,
            )
        event = payload["events_this_step"][0]
        self.assertEqual(event["event"], "heel_strike")
        self.assertAlmostEqual(event["event_time_s"], 0.021)
        self.assertAlmostEqual(event["confirmed_time_s"], 0.026)
        self.assertEqual(event["landing_sensor"], "both")
        self.assertEqual(event["hs_semantics"], "first_stable_heel_contact")
        self.assertEqual(event["semantic"], "heel_qualified_initial_contact")

    def test_toe_only_retains_an_existing_stance(self) -> None:
        fsm = self.new_fsm()
        for index in range(1, 7):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=True,
                toe_contact=False,
            )
        self.assertEqual(payload["events_this_step"][0]["event"], "heel_strike")
        for index in range(7, 57):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=True,
            )
            self.assertEqual(payload["events_this_step"], [])
            self.assertTrue(payload["in_contact"])
        self.assertEqual(payload["event_count"]["toe_off"], 0)

        for index in range(57, 63):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=False,
            )
        event = payload["events_this_step"][0]
        self.assertEqual(event["event"], "toe_off")
        self.assertAlmostEqual(event["event_time_s"], 0.057)
        self.assertAlmostEqual(event["confirmed_time_s"], 0.062)

    def test_toe_only_reset_is_partial_stance_without_synthetic_hs(self) -> None:
        fsm = self.new_fsm(toe=True)
        baseline = fsm.payload()
        self.assertTrue(baseline["in_contact"])
        self.assertEqual(baseline["events_this_step"], [])
        for index in range(1, 7):
            payload = fsm.update_sample(
                time_s=index / 1000,
                heel_contact=False,
                toe_contact=False,
            )
        event = payload["events_this_step"][0]
        self.assertEqual(event["event"], "toe_off")
        self.assertTrue(event["startup_partial_stance"])

    def test_scalar_and_batch_have_exact_event_and_latch_parity(self) -> None:
        trace: list[dict] = []
        for index in range(1, 51):
            if index <= 12:
                bits = (False, True)
            elif index <= 28:
                bits = (True, True)
            elif index <= 36:
                bits = (False, True)
            else:
                bits = (False, False)
            trace.append(sample(index / 1000, *bits))

        batch_fsm = self.new_fsm()
        batch_events: list[dict] = []
        for start in range(0, 50, 10):
            payload = batch_fsm.update_policy_step(
                time_s=(start + 10) / 1000,
                previous_time_s=start / 1000,
                sensor_samples=trace[start : start + 10],
            )
            batch_events.extend(payload["events_this_step"])

        scalar_fsm = self.new_fsm()
        scalar_events: list[dict] = []
        for index, entry in enumerate(trace, start=1):
            delivery = ((index - 1) // 10 + 1) * 0.010
            payload = scalar_fsm.update_sample(
                time_s=entry["time_s"],
                heel_contact=entry["left_heel_contact"],
                toe_contact=entry["left_toe_contact"],
                delivered_time_s=delivery,
            )
            scalar_events.extend(payload["events_this_step"])

        self.assertEqual(scalar_events, batch_events)
        scalar_payload = copy.deepcopy(scalar_fsm.payload())
        batch_payload = copy.deepcopy(batch_fsm.payload())
        for transient in (
            "events_this_step",
            "contact_state_transitions_this_step",
            "candidate_cancellations_this_step",
        ):
            scalar_payload.pop(transient)
            batch_payload.pop(transient)
        self.assertEqual(scalar_payload, batch_payload)
        json.dumps(batch_fsm.payload(), allow_nan=False)

    def test_malformed_batch_fails_atomically(self) -> None:
        fsm = self.new_fsm()
        before = fsm.payload()
        samples = [sample(i / 1000, False, True) for i in range(1, 11)]
        samples[4]["left_toe_contact"] = 1
        with self.assertRaises(TypeError):
            fsm.update_policy_step(
                time_s=0.010,
                previous_time_s=0.0,
                sensor_samples=samples,
            )
        self.assertEqual(fsm.payload(), before)


if __name__ == "__main__":
    unittest.main()
