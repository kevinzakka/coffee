"""Tests for geometry_utils."""


import numpy as np
from absl.testing import absltest

from coffee.utils import geometry_utils


class GeometryUtilsTest(absltest.TestCase):
    def test_l2_normalize(self) -> None:
        input_array = np.random.randn(4)
        output_array = geometry_utils.l2_normalize(input_array)
        self.assertTrue(np.isclose(np.linalg.norm(output_array), 1.0))

    def test_as_quaternion_xyzw(self) -> None:
        input_quaternion = np.array([1.0, 2.0, 3.0, 4.0])
        output_quaternion = geometry_utils.as_quaternion_xyzw(input_quaternion)
        expected_quaternion = np.array([2.0, 3.0, 4.0, 1.0])
        np.testing.assert_array_equal(output_quaternion, expected_quaternion)

    def test_as_quaternion_wxyz(self) -> None:
        input_quaternion = np.array([1.0, 2.0, 3.0, 4.0])
        output_quaternion = geometry_utils.as_quaternion_wxyz(input_quaternion)
        expected_quaternion = np.array([4.0, 1.0, 2.0, 3.0])
        np.testing.assert_array_equal(output_quaternion, expected_quaternion)


if __name__ == "__main__":
    absltest.main()
