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

from binary_phase_adapter import BinaryPhaseActiveAdapter  # noqa: E402
from binary_phase_adapter_v26 import (  # noqa: E402
    BinaryPhaseActiveAdapterV26,
)
from binary_phase_fsm import BinaryPhaseFSM, BinaryPhaseFSMConfig  # noqa: E402
from binary_phase_fsm_v26 import (  # noqa: E402
    HeelQualifiedBinaryPhaseFSM,
    HeelQualifiedBinaryPhaseFSMConfig,
    V26_EVENT_CONTRACT_ID,
)
from prosthetic_phase_fsm import (  # noqa: E402
    BINARY_ACTIVE_V26_ADAPTER_SOURCE,
    STANCE_AFTER_HS,
    SWING_AFTER_TO,
    WAIT_HS,
    ProstheticPhaseFSM,
    ProstheticPhaseFSMConfig,
)


def samples(start_ms: int, heel: bool, toe: bool) -> list[dict]:
    return [
        {
            "time_s": index / 1000,
            "left_heel_contact": heel,
            "left_toe_contact": toe,
        }
        for index in range(start_ms + 1, start_ms + 11)
    ]


class BinaryPhaseActiveAdapterV26Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.adapter = BinaryPhaseActiveAdapterV26()

    @staticmethod
    def new_fsms():
        return (
            HeelQualifiedBinaryPhaseFSM(
                HeelQualifiedBinaryPhaseFSMConfig(
                    event_contract_id=V26_EVENT_CONTRACT_ID
                )
            ),
            ProstheticPhaseFSM(
                ProstheticPhaseFSMConfig(
                    event_source="binary_active_v26",
                    min_stance_duration_s=0.05,
                    min_swing_duration_s=0.25,
                )
            ),
        )

    def prime(self, heel: bool = False, toe: bool = False):
        binary, phase = self.new_fsms()
        return self.adapter.prime(
            binary_fsm=binary,
            phase_fsm=phase,
            time_s=0.0,
            heel_contact=heel,
            toe_contact=toe,
        )

    def advance(
        self,
        current,
        start_ms: int,
        heel: bool,
        toe: bool,
        *,
        normal_force_bw: float = 0.0,
        in_contact: bool = False,
        sensor_samples: list[dict] | None = None,
    ):
        return self.adapter.advance(
            binary_fsm=current.binary_fsm,
            phase_fsm=current.phase_fsm,
            time_s=(start_ms + 10) / 1000,
            previous_time_s=start_ms / 1000,
            sensor_samples=(
                samples(start_ms, heel, toe)
                if sensor_samples is None
                else sensor_samples
            ),
            normal_force_bw=normal_force_bw,
            in_contact=in_contact,
        )

    def test_air_reset_has_distinct_v26_provenance(self) -> None:
        result = self.prime()
        self.assertEqual(result.binary_payload["source"], "binary_phase_fsm_v26")
        self.assertEqual(
            result.binary_payload["event_contract_id"],
            V26_EVENT_CONTRACT_ID,
        )
        self.assertEqual(result.phase_payload["event_source"], "binary_active_v26")
        self.assertEqual(result.phase_payload["state_id"], float(WAIT_HS))
        self.assertEqual(
            result.adapter_payload["adapter_source"],
            BINARY_ACTIVE_V26_ADAPTER_SOURCE,
        )
        self.assertEqual(result.left_events, [])
        json.dumps(result.adapter_payload, allow_nan=False)

    def test_toe_only_in_swing_is_diagnostic_and_not_transferred(self) -> None:
        current = self.prime()
        for start in range(0, 100, 10):
            current = self.advance(current, start, False, True)
            self.assertEqual(current.left_events, [])
            self.assertEqual(current.phase_payload["state_id"], float(WAIT_HS))
        self.assertEqual(current.binary_payload["stable_contact_state"], "TOE")
        self.assertFalse(current.binary_payload["in_contact"])
        self.assertEqual(current.phase_payload["valid_hs_count"], 0.0)

    def test_heel_after_toe_only_transfers_one_hs(self) -> None:
        current = self.prime()
        current = self.advance(current, 0, False, True)
        current = self.advance(
            current,
            10,
            True,
            True,
            normal_force_bw=0.8,
            in_contact=True,
        )
        self.assertEqual(len(current.left_events), 1)
        event = current.left_events[0]
        self.assertEqual(event["event"], "heel_strike")
        self.assertAlmostEqual(event["event_time_s"], 0.011)
        self.assertAlmostEqual(event["confirmed_time_s"], 0.016)
        self.assertAlmostEqual(event["delivered_time_s"], 0.020)
        self.assertEqual(event["source"], BINARY_ACTIVE_V26_ADAPTER_SOURCE)
        self.assertEqual(event["event_contract_id"], V26_EVENT_CONTRACT_ID)
        self.assertEqual(event["hs_semantics"], "first_stable_heel_contact")
        self.assertEqual(current.phase_payload["state_id"], float(STANCE_AFTER_HS))
        self.assertEqual(current.phase_payload["invalid_event_count"], 0.0)

    def test_primary_continuous_inputs_cannot_create_an_event(self) -> None:
        low = self.advance(
            self.prime(),
            0,
            False,
            True,
            normal_force_bw=0.0,
            in_contact=False,
        )
        high = self.advance(
            self.prime(),
            0,
            False,
            True,
            normal_force_bw=2.0,
            in_contact=True,
        )
        self.assertEqual(low.left_events, [])
        self.assertEqual(high.left_events, [])
        self.assertEqual(
            low.binary_payload["event_count"],
            high.binary_payload["event_count"],
        )

    def test_toe_only_startup_remains_partial_stance(self) -> None:
        current = self.prime(toe=True)
        self.assertEqual(current.phase_payload["state_id"], float(STANCE_AFTER_HS))
        self.assertEqual(current.phase_payload["valid_hs_count"], 0.0)
        current = self.advance(current, 0, False, False)
        self.assertEqual(len(current.left_events), 1)
        self.assertEqual(current.left_events[0]["event"], "toe_off")
        self.assertTrue(current.left_events[0]["startup_partial_stance"])
        self.assertEqual(current.phase_payload["state_id"], float(SWING_AFTER_TO))

    def test_cross_lineage_pairs_fail_closed(self) -> None:
        v26_binary, v26_phase = self.new_fsms()
        with self.assertRaises((TypeError, ValueError)):
            BinaryPhaseActiveAdapter().prime(
                binary_fsm=v26_binary,
                phase_fsm=v26_phase,
                time_s=0.0,
                heel_contact=False,
                toe_contact=False,
            )

        v20_binary = BinaryPhaseFSM(
            BinaryPhaseFSMConfig(
                event_contract_id="binary_point_v25+functional_contact_fsm_v1"
            )
        )
        v20_phase = ProstheticPhaseFSM(
            ProstheticPhaseFSMConfig(event_source="binary_active")
        )
        with self.assertRaises((TypeError, ValueError)):
            self.adapter.prime(
                binary_fsm=v20_binary,
                phase_fsm=v20_phase,
                time_s=0.0,
                heel_contact=False,
                toe_contact=False,
            )

    def test_malformed_batch_rolls_back_both_fsms(self) -> None:
        current = self.prime()
        binary_before = copy.deepcopy(current.binary_fsm.payload())
        phase_before = copy.deepcopy(current.phase_fsm.payload())
        malformed = samples(0, False, True)
        malformed[5]["time_s"] = malformed[4]["time_s"]
        with self.assertRaises(ValueError):
            self.advance(
                current,
                0,
                False,
                True,
                sensor_samples=malformed,
            )
        self.assertEqual(current.binary_fsm.payload(), binary_before)
        self.assertEqual(current.phase_fsm.payload(), phase_before)


if __name__ == "__main__":
    unittest.main()
