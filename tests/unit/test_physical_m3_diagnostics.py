"""Regression tests for M3 diagnostic artifact contracts."""

from __future__ import annotations

import unittest

import numpy as np

from tools.diagnostics.capture_physical_m3_observations import _array_contract
from tools.diagnostics.project_physical_m3_shadow import (
    _canonical_projection_action,
)


class ArrayContractTest(unittest.TestCase):
    def test_string_prompt_array_uses_lengths_not_numeric_extrema(self) -> None:
        contract = _array_contract(
            np.asarray(["pick up the object", "short"])
        )

        self.assertEqual(contract["dtype"], "<U18")
        self.assertEqual(contract["minimum_length"], 5)
        self.assertEqual(contract["maximum_length"], 18)
        self.assertNotIn("minimum", contract)

    def test_numeric_array_records_extrema(self) -> None:
        contract = _array_contract(np.asarray([1.0, -2.0]))

        self.assertEqual(contract["minimum"], -2.0)
        self.assertEqual(contract["maximum"], 1.0)


class ProjectionBoundaryTest(unittest.TestCase):
    def test_small_negative_policy_closure_is_explicitly_clipped(self) -> None:
        raw = np.arange(8, dtype=np.float64) / 10.0
        raw[7] = -0.0010503824011087418

        canonical, clipped = _canonical_projection_action(raw)

        np.testing.assert_array_equal(canonical[:7], raw[:7])
        self.assertEqual(canonical[7], 0.0)
        self.assertTrue(clipped)
        self.assertEqual(raw[7], -0.0010503824011087418)

    def test_in_range_policy_closure_is_unchanged(self) -> None:
        raw = np.zeros(8, dtype=np.float64)
        raw[7] = 0.75

        canonical, clipped = _canonical_projection_action(raw)

        np.testing.assert_array_equal(canonical, raw)
        self.assertFalse(clipped)


if __name__ == "__main__":
    unittest.main()
