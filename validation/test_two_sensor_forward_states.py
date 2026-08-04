"""Pure unit tests for the forward-state two-sensor validator helpers."""

from __future__ import annotations

import unittest

import numpy as np

from validation.validate_two_sensor_forward_states import (
    _binary_contact_metrics,
    _runtime_grid_indices,
)


class ForwardStateTwoSensorValidationTest(unittest.TestCase):
    def test_runtime_grid_selects_existing_rows_without_interpolation(self) -> None:
        native = np.round(np.arange(11.99, 12.101, 0.001), decimals=9)
        indices, contract = _runtime_grid_indices(
            native,
            sample_dt_s=0.010,
            start_s=11.99,
            end_s=12.10,
        )
        np.testing.assert_allclose(native[indices], np.arange(11.99, 12.101, 0.01))
        self.assertEqual(contract["selected_sample_count"], 12)
        self.assertLessEqual(contract["maximum_grid_alignment_error_s"], 1.0e-12)

    def test_runtime_grid_rejects_duplicate_native_rows(self) -> None:
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            _runtime_grid_indices(
                np.asarray([0.0, 0.001, 0.001, 0.002]),
                sample_dt_s=0.001,
                start_s=0.0,
                end_s=0.002,
            )

    def test_contact_metrics_report_counts_f1_and_iou(self) -> None:
        result = _binary_contact_metrics(
            np.asarray([False, True, True, False]),
            np.asarray([False, True, False, True]),
            np.ones(4, dtype=bool),
        )
        self.assertEqual(result["true_positive"], 1)
        self.assertEqual(result["false_positive"], 1)
        self.assertEqual(result["false_negative"], 1)
        self.assertAlmostEqual(result["precision"], 0.5)
        self.assertAlmostEqual(result["recall"], 0.5)
        self.assertAlmostEqual(result["f1"], 0.5)
        self.assertAlmostEqual(result["iou"], 1.0 / 3.0)

    def test_contact_metrics_respect_evaluation_mask(self) -> None:
        result = _binary_contact_metrics(
            np.asarray([True, True, False]),
            np.asarray([False, True, False]),
            np.asarray([False, True, True]),
        )
        self.assertEqual(result["samples"], 2)
        self.assertEqual(result["false_negative"], 0)
        self.assertEqual(result["f1"], 1.0)
        self.assertEqual(result["iou"], 1.0)


if __name__ == "__main__":
    unittest.main()
