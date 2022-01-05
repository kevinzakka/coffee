"""Tests for intrinsic."""

import numpy as np
from absl.testing import absltest

from coffee.sensors import Intrinsic


class IntrinsicTest(absltest.TestCase):
    def setUp(self) -> None:
        self.intr = Intrinsic(width=640, height=480, fx=450, fy=450, cx=320, cy=240)

    def test_intr_immutable(self) -> None:
        with self.assertRaises(AttributeError):
            self.intr.height = 0  # type: ignore

    def test_intr_matrix(self) -> None:
        expected = np.array(
            [
                [self.intr.fx, 0, self.intr.cx],
                [0, self.intr.fy, self.intr.cy],
                [0, 0, 1],
            ]
        )
        np.testing.assert_equal(self.intr.matrix, expected)

    def test_intr_resolution(self) -> None:
        assert self.intr.resolution == (self.intr.width, self.intr.height)

    def test_intr_level(self) -> None:
        # Level 0 should give the same parameters.
        expected = self.intr.matrix
        actual = self.intr.level(0).matrix
        np.testing.assert_equal(actual, expected)

        # Level 1 should be half the size, level 2 should be 1/4 the size, level 3
        # should be 1/8 the size, and so on.
        for level in range(1, 4):
            expected = Intrinsic(
                width=self.intr.width / (2 ** level),
                height=self.intr.height / (2 ** level),
                fx=self.intr.fx / (2 ** level),
                fy=self.intr.fy / (2 ** level),
                cx=(self.intr.cx + 0.5) / (2 ** level) - 0.5,
                cy=(self.intr.cy + 0.5) / (2 ** level) - 0.5,
            ).matrix
            actual = self.intr.level(level).matrix
            np.testing.assert_equal(actual, expected)


if __name__ == "__main__":
    absltest.main()
