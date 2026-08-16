"""Pure-Python contract tests for the V25/V20 ``binary_active`` adapter.

The adapter is a transactional bridge: :class:`BinaryPhaseFSM` remains the
event detector, while :class:`ProstheticPhaseFSM` remains the only owner of the
actor-visible gait state.  These tests deliberately avoid the OpenSim runner
and environment so reset, event ordering, timestamps, and rollback semantics
can be checked exactly.
"""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ROOT = REPO_ROOT / "Trajectory Generator"
if str(TRAJECTORY_ROOT) not in sys.path:
    sys.path.insert(0, str(TRAJECTORY_ROOT))

from binary_phase_adapter import BinaryPhaseActiveAdapter  # noqa: E402
from binary_phase_fsm import BinaryPhaseFSM, BinaryPhaseFSMConfig  # noqa: E402
from prosthetic_phase_fsm import (  # noqa: E402
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    WAIT_HS,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)


ACTIVE_CONTRACT = "binary_point_v25+functional_contact_fsm_v1"
ACTIVE_SOURCE = "v25_fsm_v20"


def detector_samples(
    start_ms: int,
    *,
    heel_contact: bool,
    toe_contact: bool,
) -> list[dict[str, object]]:
    """Return the ten samples in ``(start_ms, start_ms + 10]``."""

    return [
        {
            "time_s": index / 1000.0,
            "left_heel_contact": heel_contact,
            "left_toe_contact": toe_contact,
        }
        for index in range(start_ms + 1, start_ms + 11)
    ]


class BinaryPhaseActiveAdapterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.adapter = BinaryPhaseActiveAdapter()

    @staticmethod
    def new_fsms() -> tuple[BinaryPhaseFSM, ProstheticPhaseFSM]:
        binary_fsm = BinaryPhaseFSM(
            BinaryPhaseFSMConfig(event_contract_id=ACTIVE_CONTRACT)
        )
        phase_fsm = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(
                event_source="binary_active",
                min_stance_duration_s=0.05,
                min_swing_duration_s=0.20,
            )
        )
        return binary_fsm, phase_fsm

    def prime(self, heel: bool, toe: bool, *, time_s: float = 0.0):
        binary_fsm, phase_fsm = self.new_fsms()
        return self.adapter.prime(
            binary_fsm=binary_fsm,
            phase_fsm=phase_fsm,
            time_s=time_s,
            heel_contact=heel,
            toe_contact=toe,
        )

    def advance(
        self,
        current,
        start_ms: int,
        *,
        heel: bool,
        toe: bool,
        normal_force_bw: float | None = None,
        in_contact: bool | None = None,
        sensor_samples: list[dict[str, object]] | None = None,
    ):
        if normal_force_bw is None:
            normal_force_bw = 0.8 if heel or toe else 0.0
        if in_contact is None:
            in_contact = bool(heel or toe)
        return self.adapter.advance(
            binary_fsm=current.binary_fsm,
            phase_fsm=current.phase_fsm,
            time_s=(start_ms + 10) / 1000.0,
            previous_time_s=start_ms / 1000.0,
            sensor_samples=(
                detector_samples(
                    start_ms,
                    heel_contact=heel,
                    toe_contact=toe,
                )
                if sensor_samples is None
                else sensor_samples
            ),
            normal_force_bw=normal_force_bw,
            in_contact=in_contact,
        )

    def assert_result_consistent(self, result) -> None:
        self.assertEqual(result.binary_payload, result.binary_fsm.payload())
        self.assertEqual(result.phase_payload, result.phase_fsm.payload())
        self.assertIsInstance(result.left_events, (list, tuple))
        self.assertIsInstance(result.adapter_payload, dict)
        json.dumps(
            {
                "binary": result.binary_payload,
                "phase": result.phase_payload,
                "left_events": list(result.left_events),
                "adapter": result.adapter_payload,
            },
            allow_nan=False,
        )

    def assert_active_event(
        self,
        event: dict,
        *,
        name: str,
        event_time_s: float,
        confirmed_time_s: float,
        delivered_time_s: float,
    ) -> None:
        self.assertEqual(event["side"], "left")
        self.assertEqual(event["event"], name)
        self.assertEqual(event["source"], ACTIVE_SOURCE)
        self.assertEqual(event["event_contract_id"], ACTIVE_CONTRACT)
        self.assertAlmostEqual(event["event_time_s"], event_time_s, places=12)
        self.assertAlmostEqual(
            event["confirmed_time_s"], confirmed_time_s, places=12
        )
        self.assertAlmostEqual(
            event["delivered_time_s"], delivered_time_s, places=12
        )

    def test_active_reset_air_primes_both_fsms_without_event(self) -> None:
        result = self.prime(False, False)
        self.assert_result_consistent(result)

        binary = result.binary_payload
        phase = result.phase_payload
        self.assertEqual(binary["event_contract_id"], ACTIVE_CONTRACT)
        self.assertEqual(binary["stable_contact_state"], "AIR")
        self.assertFalse(binary["in_contact"])
        self.assertEqual(binary["events_this_step"], [])
        self.assertEqual(phase["event_source"], "binary_active")
        self.assertEqual(phase["state_id"], float(WAIT_HS))
        self.assertEqual(phase["valid_hs_count"], 0.0)
        self.assertEqual(phase["valid_to_count"], 0.0)
        self.assertEqual(phase["accepted_transitions_this_step"], [])
        self.assertEqual(result.left_events, [])
        self.assertEqual(result.phase_fsm.observation()["phase_expected_hs"], 1.0)

    def test_active_reset_contact_primes_partial_stance_without_synthetic_hs(
        self,
    ) -> None:
        for heel, toe, word in (
            (True, False, "HEEL"),
            (False, True, "TOE"),
            (True, True, "BOTH"),
        ):
            with self.subTest(word=word):
                result = self.prime(heel, toe)
                self.assert_result_consistent(result)
                binary = result.binary_payload
                phase = result.phase_payload

                self.assertEqual(binary["stable_contact_state"], word)
                self.assertTrue(binary["in_contact"])
                self.assertEqual(binary["events_this_step"], [])
                self.assertEqual(phase["state_id"], float(STANCE_AFTER_HS))
                self.assertEqual(phase["sensor_partial_stance_active"], 1.0)
                self.assertEqual(phase["last_valid_hs_time_s"], -1.0)
                self.assertEqual(phase["valid_hs_count"], 0.0)
                self.assertEqual(phase["valid_cycle_count"], 0.0)
                self.assertEqual(phase["accepted_transitions_this_step"], [])
                self.assertEqual(phase["phase_event_progress_score"], 0.0)
                self.assertEqual(phase["pending_cycle_credit"], 0.0)
                self.assertEqual(phase["cycle_progress_credit"], 0.0)
                self.assertEqual(result.left_events, [])

    def test_active_partial_stance_leading_to_is_accepted(self) -> None:
        for heel, toe, word in (
            (True, False, "HEEL"),
            (False, True, "TOE"),
            (True, True, "BOTH"),
        ):
            with self.subTest(word=word):
                primed = self.prime(heel, toe)
                result = self.advance(
                    primed,
                    0,
                    heel=False,
                    toe=False,
                )
                self.assert_result_consistent(result)

                detector_events = result.binary_payload["events_this_step"]
                self.assertEqual(len(detector_events), 1)
                self.assertEqual(detector_events[0]["event"], "toe_off")
                self.assertTrue(detector_events[0]["startup_partial_stance"])
                self.assertEqual(len(result.left_events), 1)
                self.assert_active_event(
                    result.left_events[0],
                    name="toe_off",
                    event_time_s=0.001,
                    confirmed_time_s=0.006,
                    delivered_time_s=0.010,
                )
                self.assertTrue(
                    result.left_events[0]["startup_partial_stance"]
                )

                phase = result.phase_payload
                self.assertEqual(phase["state_id"], float(SWING_AFTER_TO))
                self.assertEqual(phase["valid_hs_count"], 0.0)
                self.assertEqual(phase["valid_to_count"], 1.0)
                self.assertEqual(phase["valid_cycle_count"], 0.0)
                self.assertEqual(phase["invalid_event_count"], 0.0)
                self.assertEqual(phase["phase_event_progress_score"], 0.0)
                self.assertEqual(phase["pending_cycle_credit"], 0.0)
                transition = phase["accepted_transitions_this_step"]
                self.assertEqual(len(transition), 1)
                self.assertEqual(transition[0]["event"], "toe_off")
                self.assertEqual(transition[0]["segment_valid"], 0.0)

                # The first HS after the partial stance opens the first full
                # stance; it must not fabricate a completed HS-to-HS cycle.
                current = result
                for start_ms in range(10, 200, 10):
                    current = self.advance(
                        current,
                        start_ms,
                        heel=False,
                        toe=False,
                    )
                current = self.advance(
                    current,
                    200,
                    heel=True,
                    toe=False,
                )
                self.assertEqual(len(current.left_events), 1)
                self.assertEqual(current.left_events[0]["event"], "heel_strike")
                self.assertEqual(current.phase_payload["valid_hs_count"], 1.0)
                self.assertEqual(current.phase_payload["valid_to_count"], 1.0)
                self.assertEqual(current.phase_payload["valid_cycle_count"], 0.0)
                self.assertEqual(current.phase_payload["last_period_s"], 0.0)
                self.assertEqual(current.phase_payload["previous_period_s"], 0.0)
                self.assertEqual(current.phase_payload["invalid_event_count"], 0.0)

    def test_active_air_to_contact_hs_for_every_landing_word(self) -> None:
        for heel, toe, leader in (
            (True, False, "heel"),
            (False, True, "toe"),
            (True, True, "both"),
        ):
            with self.subTest(leader=leader):
                primed = self.prime(False, False)
                result = self.advance(
                    primed,
                    0,
                    heel=heel,
                    toe=toe,
                )
                self.assert_result_consistent(result)

                detector_event = result.binary_payload["events_this_step"][0]
                self.assertEqual(detector_event["event"], "heel_strike")
                self.assertEqual(detector_event["landing_sensor"], leader)
                self.assertEqual(len(result.left_events), 1)
                self.assert_active_event(
                    result.left_events[0],
                    name="heel_strike",
                    event_time_s=0.001,
                    confirmed_time_s=0.006,
                    delivered_time_s=0.010,
                )

                phase = result.phase_payload
                self.assertEqual(phase["state_id"], float(STANCE_AFTER_HS))
                self.assertEqual(phase["valid_hs_count"], 1.0)
                self.assertEqual(phase["valid_to_count"], 0.0)
                self.assertEqual(phase["invalid_event_count"], 0.0)
                transition = phase["accepted_transitions_this_step"]
                self.assertEqual(len(transition), 1)
                self.assertEqual(transition[0]["event"], "heel_strike")
                self.assertAlmostEqual(
                    transition[0]["event_time_s"], 0.001, places=12
                )

    def test_actor_phase_observation_contract_matches_legacy_exactly(self) -> None:
        active = self.prime(False, False)
        legacy = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(
                event_source="legacy_events",
                min_stance_duration_s=0.05,
                min_swing_duration_s=0.20,
            )
        )
        self.assertEqual(active.phase_fsm.observation(), legacy.observation())

        active = self.advance(active, 0, heel=True, toe=False)
        legacy.update(
            time_s=0.010,
            events=(
                {
                    "side": "left",
                    "event": "heel_strike",
                    "time": 0.001,
                    "event_time_s": 0.001,
                    "confirmed_time_s": 0.006,
                    "delivered_time_s": 0.010,
                },
            ),
            normal_force_bw=0.8,
            in_contact=True,
        )
        active_observation = active.phase_fsm.observation()
        legacy_observation = legacy.observation()
        self.assertEqual(tuple(active_observation), tuple(legacy_observation))
        self.assertEqual(len(active_observation), 8)
        self.assertEqual(active_observation, legacy_observation)

    def test_active_normal_hs_to_hs_cycle_exact_order(self) -> None:
        current = self.prime(False, False)
        observed_events: list[dict] = []

        # HS onset 1 ms, confirmation 6 ms, delivery 10 ms.
        current = self.advance(current, 0, heel=True, toe=False)
        observed_events.extend(current.left_events)

        # Keep stance through 60 ms so the physical TO passes the 50 ms gate.
        for start_ms in range(10, 60, 10):
            current = self.advance(current, start_ms, heel=True, toe=True)
            observed_events.extend(current.left_events)

        # TO onset 61 ms, confirmation 66 ms, delivery 70 ms.
        current = self.advance(current, 60, heel=False, toe=False)
        observed_events.extend(current.left_events)

        # Keep swing through 270 ms, then establish the next HS at 271 ms.
        for start_ms in range(70, 270, 10):
            current = self.advance(current, start_ms, heel=False, toe=False)
            observed_events.extend(current.left_events)
        current = self.advance(current, 270, heel=True, toe=False)
        observed_events.extend(current.left_events)

        self.assertEqual(
            [event["event"] for event in observed_events],
            ["heel_strike", "toe_off", "heel_strike"],
        )
        expected_times = (
            (0.001, 0.006, 0.010),
            (0.061, 0.066, 0.070),
            (0.271, 0.276, 0.280),
        )
        for event, (onset, confirmed, delivered) in zip(
            observed_events, expected_times
        ):
            self.assert_active_event(
                event,
                name=event["event"],
                event_time_s=onset,
                confirmed_time_s=confirmed,
                delivered_time_s=delivered,
            )

        self.assertEqual(current.binary_payload["cycle_count"], 1)
        self.assertEqual(current.phase_payload["state_id"], float(STANCE_AFTER_HS))
        self.assertEqual(current.phase_payload["valid_hs_count"], 2.0)
        self.assertEqual(current.phase_payload["valid_to_count"], 1.0)
        self.assertEqual(current.phase_payload["valid_cycle_count"], 1.0)
        self.assertEqual(current.phase_payload["invalid_event_count"], 0.0)
        self.assertAlmostEqual(
            current.phase_payload["last_period_s"], 0.270, places=12
        )

    def test_malformed_batch_rolls_back_both_fsms(self) -> None:
        malformed_batches: list[tuple[str, list[dict[str, object]]]] = []

        incomplete = detector_samples(
            0, heel_contact=False, toe_contact=False
        )[:-1]
        malformed_batches.append(("incomplete", incomplete))

        non_bool = detector_samples(0, heel_contact=False, toe_contact=False)
        non_bool[4]["left_heel_contact"] = 1
        malformed_batches.append(("non_bool", non_bool))

        duplicate = detector_samples(0, heel_contact=False, toe_contact=False)
        duplicate[6]["time_s"] = duplicate[5]["time_s"]
        malformed_batches.append(("duplicate", duplicate))

        for label, malformed in malformed_batches:
            with self.subTest(label=label):
                primed = self.prime(False, False)
                binary_before = copy.deepcopy(primed.binary_fsm.payload())
                phase_before = copy.deepcopy(primed.phase_fsm.payload())
                with self.assertRaises((TypeError, ValueError)):
                    self.advance(
                        primed,
                        0,
                        heel=False,
                        toe=False,
                        sensor_samples=malformed,
                    )
                self.assertEqual(primed.binary_fsm.payload(), binary_before)
                self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_active_policy_cadence_is_exactly_ten_milliseconds(self) -> None:
        for duration_ms in (1, 5, 9, 11):
            with self.subTest(duration_ms=duration_ms):
                primed = self.prime(False, False)
                binary_before = copy.deepcopy(primed.binary_fsm.payload())
                phase_before = copy.deepcopy(primed.phase_fsm.payload())
                batch = [
                    {
                        "time_s": index / 1000.0,
                        "left_heel_contact": False,
                        "left_toe_contact": False,
                    }
                    for index in range(1, duration_ms + 1)
                ]

                with self.assertRaisesRegex(ValueError, "exactly 10 ms"):
                    self.adapter.advance(
                        binary_fsm=primed.binary_fsm,
                        phase_fsm=primed.phase_fsm,
                        time_s=duration_ms / 1000.0,
                        previous_time_s=0.0,
                        sensor_samples=batch,
                        normal_force_bw=0.0,
                        in_contact=False,
                    )

                self.assertEqual(primed.binary_fsm.payload(), binary_before)
                self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_nonfinite_active_config_or_payload_is_rejected(self) -> None:
        binary_fsm = BinaryPhaseFSM(
            BinaryPhaseFSMConfig(event_contract_id=ACTIVE_CONTRACT)
        )
        phase_fsm = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(
                event_source="binary_active",
                hs_event_credit=float("nan"),
            )
        )
        with self.assertRaisesRegex(ValueError, "hs_event_credit must be finite"):
            self.adapter.prime(
                binary_fsm=binary_fsm,
                phase_fsm=phase_fsm,
                time_s=0.0,
                heel_contact=False,
                toe_contact=False,
            )

        primed = self.prime(False, False)
        primed.phase_fsm.pending_cycle_credit = float("inf")
        binary_before = copy.deepcopy(primed.binary_fsm.payload())
        with self.assertRaisesRegex(ValueError, "strict JSON"):
            self.advance(primed, 0, heel=False, toe=False)
        self.assertEqual(primed.binary_fsm.payload(), binary_before)

    def test_active_event_timestamp_contract_is_mandatory_and_exact(self) -> None:
        primed = self.prime(False, False)
        phase_before = copy.deepcopy(primed.phase_fsm.payload())
        base_event = {
            "side": "left",
            "event": "heel_strike",
            "event_time_s": 0.001,
            "confirmed_time_s": 0.006,
            "delivered_time_s": 0.010,
            "source": ACTIVE_SOURCE,
            "event_contract_id": ACTIVE_CONTRACT,
        }
        malformed_events = []
        missing_confirmation = dict(base_event)
        missing_confirmation.pop("confirmed_time_s")
        malformed_events.append(missing_confirmation)
        wrong_latency = dict(base_event)
        wrong_latency["confirmed_time_s"] = 0.005
        malformed_events.append(wrong_latency)
        replayed_confirmation = dict(base_event)
        replayed_confirmation["event_time_s"] = -0.005
        replayed_confirmation["confirmed_time_s"] = 0.0
        malformed_events.append(replayed_confirmation)

        for event in malformed_events:
            with self.subTest(event=event):
                with self.assertRaises(ValueError):
                    primed.phase_fsm.update_from_binary_events(
                        time_s=0.010,
                        previous_time_s=0.0,
                        events=(event,),
                        normal_force_bw=0.8,
                        in_contact=True,
                    )
                self.assertEqual(primed.phase_fsm.payload(), phase_before)

        binary_before = copy.deepcopy(primed.binary_fsm.payload())
        real_binary_update = BinaryPhaseFSM.update_policy_step

        def drift_confirmation(instance, *args, **kwargs):
            payload = real_binary_update(instance, *args, **kwargs)
            payload["events_this_step"][0]["confirmed_time_s"] = 0.005
            return payload

        with mock.patch.object(
            BinaryPhaseFSM,
            "update_policy_step",
            autospec=True,
            side_effect=drift_confirmation,
        ):
            with self.assertRaisesRegex(ValueError, "must equal 5 ms"):
                self.advance(primed, 0, heel=True, toe=False)
        self.assertEqual(primed.binary_fsm.payload(), binary_before)
        self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_absolute_clock_endpoint_roundoff_is_accepted_end_to_end(self) -> None:
        absolute_start = 11.99 + 1.756870983805102
        policy_previous = absolute_start
        sensor_previous = absolute_start
        for _step in range(449):
            policy_previous += 0.010
            for _sample in range(10):
                sensor_previous += 0.001

        boundary = policy_previous + 0.010
        samples = []
        for index in range(10):
            sensor_previous += 0.001
            contact = index >= 4
            samples.append(
                {
                    "time_s": sensor_previous,
                    "left_heel_contact": contact,
                    "left_toe_contact": False,
                }
            )

        primed = self.prime(
            False,
            False,
            time_s=float(samples[0]["time_s"]) - 0.001,
        )
        result = self.adapter.advance(
            binary_fsm=primed.binary_fsm,
            phase_fsm=primed.phase_fsm,
            time_s=boundary,
            previous_time_s=policy_previous,
            sensor_samples=samples,
            normal_force_bw=0.8,
            in_contact=True,
        )

        self.assertEqual(len(result.left_events), 1)
        event = result.left_events[0]
        self.assertEqual(event["event"], "heel_strike")
        endpoint_roundoff = (
            event["confirmed_time_s"] - event["delivered_time_s"]
        )
        self.assertGreater(endpoint_roundoff, 1e-12)
        self.assertLess(endpoint_roundoff, 1e-9)
        self.assertEqual(
            event["confirmed_time_s"],
            samples[-1]["time_s"],
        )
        self.assertEqual(event["delivered_time_s"], boundary)
        transition = result.phase_payload["accepted_transitions_this_step"]
        self.assertEqual(len(transition), 1)
        self.assertEqual(
            transition[0]["confirmed_time_s"],
            event["confirmed_time_s"],
        )
        self.assertEqual(
            transition[0]["delivered_time_s"],
            event["delivered_time_s"],
        )

    def test_binary_active_causality_drift_above_one_nanosecond_fails(self) -> None:
        primed = self.prime(False, False)
        phase_before = copy.deepcopy(primed.phase_fsm.payload())
        confirmed = 0.010 + 2e-9
        event = {
            "side": "left",
            "event": "heel_strike",
            "event_time_s": confirmed - 0.005,
            "confirmed_time_s": confirmed,
            "delivered_time_s": 0.010,
            "source": ACTIVE_SOURCE,
            "event_contract_id": ACTIVE_CONTRACT,
        }

        with self.assertRaisesRegex(ValueError, "finite and causal"):
            primed.phase_fsm.update_from_binary_events(
                time_s=0.010,
                previous_time_s=0.0,
                events=(event,),
                normal_force_bw=0.8,
                in_contact=True,
            )
        self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_non_active_event_sources_keep_picosecond_causality(self) -> None:
        confirmed = 0.010 + 5e-10
        event = {
            "event_time_s": confirmed - 0.005,
            "confirmed_time_s": confirmed,
            "delivered_time_s": 0.010,
        }
        for source in ("legacy_events", "two_sensor"):
            with self.subTest(source=source):
                phase_fsm = ProstheticPhaseFSM(
                    ProstheticPhaseFSMConfig(event_source=source)
                )
                with self.assertRaisesRegex(ValueError, "finite and causal"):
                    phase_fsm._event_confirmation_delivery(
                        event,
                        float(event["event_time_s"]),
                    )

    def test_rejected_actor_event_error_has_strict_json_context(self) -> None:
        event = {
            "side": "left",
            "event": "heel_strike",
            "source": ACTIVE_SOURCE,
            "event_contract_id": ACTIVE_CONTRACT,
            "event_time_s": 18.4938709838069,
            "confirmed_time_s": 18.498870983806906,
            "delivered_time_s": 18.506870983804248,
        }
        phase_payload = {
            "state_id": float(SWING_AFTER_TO),
            "state_name": "SWING_AFTER_TO",
            "event_source": "binary_active",
            "invalid_event_this_step": 1.0,
            "invalid_event_type": "hs_too_early_after_to",
        }

        with self.assertRaisesRegex(
            ValueError,
            "^Actor FSM rejected a V20 active event:",
        ) as captured:
            BinaryPhaseActiveAdapter._validate_transfer(
                (event,),
                phase_payload,
            )

        encoded = str(captured.exception).split(": ", maxsplit=1)[1]
        diagnostic = json.loads(encoded)
        self.assertEqual(
            diagnostic,
            {
                "invalid_event_type": "hs_too_early_after_to",
                "state_name": "SWING_AFTER_TO",
                "adapted_events": [event],
            },
        )
        self.assertNotIn("NaN", encoded)
        self.assertNotIn("Infinity", encoded)

        nonfinite = copy.deepcopy(event)
        nonfinite["confirmed_time_s"] = float("nan")
        with self.assertRaisesRegex(ValueError, "must be finite"):
            BinaryPhaseActiveAdapter._validate_transfer(
                (nonfinite,),
                phase_payload,
            )

    def test_generic_update_cannot_inject_legacy_left_event_in_active_mode(
        self,
    ) -> None:
        primed = self.prime(False, False)
        phase_before = copy.deepcopy(primed.phase_fsm.payload())

        with self.assertRaisesRegex(ValueError, "generic or legacy"):
            primed.phase_fsm.update(
                time_s=0.010,
                events=(
                    {
                        "side": "left",
                        "event": "heel_strike",
                        "time": 0.001,
                        "source": "legacy_analog",
                    },
                ),
                normal_force_bw=0.8,
                in_contact=True,
            )

        self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_actor_fsm_cursor_must_match_v20_previous_boundary(self) -> None:
        primed = self.prime(False, False)
        current = self.advance(primed, 0, heel=False, toe=False)
        binary_before = copy.deepcopy(current.binary_fsm.payload())
        stale_phase_before = copy.deepcopy(primed.phase_fsm.payload())

        with self.assertRaisesRegex(ValueError, "boundary is discontinuous"):
            self.adapter.advance(
                binary_fsm=current.binary_fsm,
                phase_fsm=primed.phase_fsm,
                time_s=0.020,
                previous_time_s=0.010,
                sensor_samples=detector_samples(
                    10,
                    heel_contact=False,
                    toe_contact=False,
                ),
                normal_force_bw=0.0,
                in_contact=False,
            )

        self.assertEqual(current.binary_fsm.payload(), binary_before)
        self.assertEqual(primed.phase_fsm.payload(), stale_phase_before)

    def test_forbidden_actor_fsm_states_are_rejected_without_advancing_v20(
        self,
    ) -> None:
        for forbidden_state in (3, 5, 999):
            with self.subTest(state_id=forbidden_state):
                primed = self.prime(False, False)
                primed.phase_fsm.state_id = forbidden_state
                binary_before = copy.deepcopy(primed.binary_fsm.payload())
                phase_before = copy.deepcopy(primed.phase_fsm.payload())

                with self.assertRaisesRegex(ValueError, "allowed runtime state"):
                    self.advance(primed, 0, heel=False, toe=False)

                self.assertEqual(primed.binary_fsm.payload(), binary_before)
                self.assertEqual(primed.phase_fsm.payload(), phase_before)

    def test_downstream_phase_failure_rolls_back_binary_and_phase_fsms(self) -> None:
        primed = self.prime(False, False)
        binary_before = copy.deepcopy(primed.binary_fsm.payload())
        phase_before = copy.deepcopy(primed.phase_fsm.payload())
        real_update = ProstheticPhaseFSM.update_from_binary_events

        def mutate_then_fail(instance, *args, **kwargs):
            real_update(instance, *args, **kwargs)
            raise RuntimeError("injected downstream phase failure")

        with mock.patch.object(
            ProstheticPhaseFSM,
            "update_from_binary_events",
            autospec=True,
            side_effect=mutate_then_fail,
        ):
            with self.assertRaisesRegex(
                RuntimeError, "injected downstream phase failure"
            ):
                self.advance(primed, 0, heel=True, toe=False)

        self.assertEqual(primed.binary_fsm.payload(), binary_before)
        self.assertEqual(primed.phase_fsm.payload(), phase_before)


if __name__ == "__main__":
    unittest.main()
