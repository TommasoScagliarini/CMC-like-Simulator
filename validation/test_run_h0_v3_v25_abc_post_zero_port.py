from __future__ import annotations

import unittest
from unittest import mock

import numpy as np

from validation import run_h0_v3_v25_abc_post_zero_port as driver


class PostZeroPortDriverTests(unittest.TestCase):
    def test_destination_matrix_is_exact_and_disjoint(self) -> None:
        paths = {
            driver.destination(mode, item["case_id"])
            for mode in driver.contract.MODES
            for item in driver.contract.CASES
        }
        self.assertEqual(len(paths), 18)
        self.assertTrue(all(path.parent == driver.RUN_ROOT for path in paths))

    def test_frozen_noise_tapes_have_exact_contract(self) -> None:
        for item in driver.contract.CASES:
            with self.subTest(case=item["case_id"]):
                values = driver._load_noise(item, np)
                self.assertEqual(values.dtype, np.dtype("float32"))
                self.assertEqual(values.shape, (500, 2))
                self.assertTrue(np.all(np.isfinite(values)))

    def test_build_env_config_pins_morphology_and_v25_modes(self) -> None:
        case = driver.condition("deterministic_offset_nominal")
        for mode in driver.contract.MODES:
            expected = {
                "reward": {"morphology_weight": 0.0},
                "phase_fsm_input_mode": "legacy_events",
                "online_grf_applied_sides": ["left"],
                "binary_phase_fsm_mode": driver.contract.MODES[mode][
                    "binary_phase_fsm_mode"
                ],
            }
            with self.subTest(mode=mode), mock.patch.object(
                driver.legacy, "build_env_config", return_value=expected
            ) as delegated:
                observed = driver.build_env_config(mode=mode, case=case)
                self.assertEqual(observed, expected)
                delegated.assert_called_once()

    def test_unknown_condition_and_mode_fail_closed(self) -> None:
        with self.assertRaises(driver.PostZeroPortExecutionError):
            driver.condition("unknown")
        with self.assertRaises(driver.PostZeroPortExecutionError):
            driver.destination("D", driver.contract.CASES[0]["case_id"])


if __name__ == "__main__":
    unittest.main()
