"""Tests for body."""

from typing import Optional, Tuple

import numpy as np
from absl.testing import absltest, parameterized

from coffee import body
from coffee.hints import Array
from coffee.utils import testing_utils
from coffee.utils.geometry_utils import quaternion_equal

# Quaternions for special rotations.
_NO_ROTATION = (1, 0, 0, 0)
_NINETY_DEGREES_ABOUT_X = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
_NINETY_DEGREES_ABOUT_Y = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
_NINETY_DEGREES_ABOUT_Z = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
_FORTYFIVE_DEGREES_ABOUT_X = np.array([np.cos(np.pi / 8), np.sin(np.pi / 8), 0.0, 0.0])

# Triplets of starting rotation, rotation by, and final rotation.
_TEST_ROTATIONS = [
    (
        None,
        _NO_ROTATION,
        _NO_ROTATION,
    ),
    (
        _NO_ROTATION,
        _NINETY_DEGREES_ABOUT_Z,
        _NINETY_DEGREES_ABOUT_Z,
    ),
    (
        _FORTYFIVE_DEGREES_ABOUT_X,
        _NINETY_DEGREES_ABOUT_Y,
        np.array([0.65328, 0.2706, 0.65328, -0.2706]),
    ),
]


class _TestBody(body.Body):
    """Subclass of Body for testing."""

    def __init__(self, body_id: int, pb_client: body.BulletClient) -> None:
        super().__init__(body_id, pb_client)


class BodyTest(testing_utils.BulletMultiDirectParameterizedTestCase):
    def setUp(self) -> None:
        super().setUp()

        body_id = self.client.load_urdf("sphere.urdf", useFixedBase=False)
        self.body = _TestBody(body_id=body_id, pb_client=self.client)

    def test_raises_body_id_negative(self) -> None:
        # Mimic failing to load a URDF which returns a negative body id.
        with self.assertRaises(ValueError):
            _TestBody(body_id=-1, pb_client=self.client)

    @parameterized.parameters(
        *testing_utils.param_product(
            position=[None, [1.0, 0.0, -1.0]],
            quaternion=[None, _FORTYFIVE_DEGREES_ABOUT_X, _NINETY_DEGREES_ABOUT_Z],
        )
    )
    def test_set_get_pose(
        self, position: Optional[Array], quaternion: Optional[Array]
    ) -> None:
        if quaternion is None:
            ground_truth_quaternion = _NO_ROTATION
        else:
            ground_truth_quaternion = quaternion  # type: ignore
        if position is None:
            ground_truth_position = np.array([0.0, 0.0, 0.0])
        else:
            ground_truth_position = position  # type: ignore

        self.body.set_pose(position=position, quaternion=quaternion)

        actual_position, actual_quaternion = self.body.get_pose()

        np.testing.assert_array_equal(actual_position, ground_truth_position)
        self.assertTrue(quaternion_equal(actual_quaternion, ground_truth_quaternion))

    @parameterized.parameters(
        *testing_utils.param_product(
            original_position=[[-2, -1, -1.0], [1.0, 0.0, -1.0]],
            position=[None, [1.0, 0.0, -1.0]],
            quaternion_triplet=_TEST_ROTATIONS,
        )
    )
    def test_shift_pose(
        self,
        original_position: Array,
        position: Optional[Array],
        quaternion_triplet: Tuple[Array, Array, Array],
    ) -> None:
        original_quaternion, quaternion, expected_quaternion = quaternion_triplet

        # Set the original position.
        self.body.set_pose(position=original_position, quaternion=original_quaternion)

        if position is None:
            ground_truth_position = original_position
        else:
            ground_truth_position = original_position + np.array(position)
        self.body.shift_pose(position=position, quaternion=quaternion)
        actual_position, actual_quaternion = self.body.get_pose()
        np.testing.assert_array_equal(actual_position, ground_truth_position)
        self.assertTrue(quaternion_equal(actual_quaternion, expected_quaternion))

    def test_set_pose_preserves_velocity(self) -> None:
        # Check that we start with 0 velocity.
        current_linear, current_angular = self.body.get_velocity()
        np.testing.assert_almost_equal(current_linear, np.zeros(3))
        np.testing.assert_almost_equal(current_angular, np.zeros(3))

        # Set random velocities.
        linear_velocity = np.random.uniform(-1.0, 1.0, size=3)
        angular_velocity = np.random.uniform(-1.0, 1.0, size=3)
        self.body.set_velocity(linear_velocity, angular_velocity)

        # Check that velocities were correctly set.
        current_linear, current_angular = self.body.get_velocity()
        np.testing.assert_almost_equal(current_linear, linear_velocity)
        np.testing.assert_almost_equal(current_angular, angular_velocity)

        # Set random pose.
        position = np.random.uniform(-1.0, 1.0, size=3)
        self.body.set_pose(position=position)

        # Check that velocities were correctly set.
        current_linear, current_angular = self.body.get_velocity()
        np.testing.assert_almost_equal(current_linear, linear_velocity)
        np.testing.assert_almost_equal(current_angular, angular_velocity)

    @parameterized.named_parameters(("rotation_off", False), ("rotation_on", True))
    def test_shift_pose_with_velocity(self, rotate_velocity: bool) -> None:
        # Set the original position.
        self.body.set_pose(position=[0.0, 0.0, 0.0])

        # Set velocity in y dim.
        self.body.set_velocity(linear=[0.0, 1.0, 0.0])

        # Rotate the body around the z-axis.
        self.body.shift_pose(
            quaternion=_NINETY_DEGREES_ABOUT_Z, rotate_velocity=rotate_velocity
        )

        self.client.step()
        current_position, _ = self.body.get_pose()

        if rotate_velocity:
            # Should not have moved in the y-direction.
            np.testing.assert_array_almost_equal(current_position[1], 0.0)
        else:
            # Should not have moved in the x-direction.
            np.testing.assert_array_almost_equal(current_position[0], 0.0)

    def test_set_velocity_one_arg_preserves_other(self) -> None:
        # Set the original position.
        self.body.set_pose(position=[0.0, 0.0, 0.0])

        # Set random linear and angular velocities.
        linear_vel = np.random.uniform(-1.0, 1.0, size=3)
        angular_vel = np.random.uniform(-1.0, 1.0, size=3)
        self.body.set_velocity(linear=linear_vel, angular=angular_vel)

        # Set linear to 0.
        self.body.set_velocity(linear=np.zeros(3))
        current_linear_vel, current_angular_vel = self.body.get_velocity()
        np.testing.assert_equal(current_linear_vel, np.zeros(3))
        np.testing.assert_equal(current_angular_vel, angular_vel)

        # Again, set random linear and angular velocities.
        linear_vel = np.random.uniform(-1.0, 1.0, size=3)
        angular_vel = np.random.uniform(-1.0, 1.0, size=3)
        self.body.set_velocity(linear=linear_vel, angular=angular_vel)

        # Set angular to 0.
        self.body.set_velocity(angular=np.zeros(3))
        current_linear_vel, current_angular_vel = self.body.get_velocity()
        np.testing.assert_equal(current_linear_vel, linear_vel)
        np.testing.assert_equal(current_angular_vel, np.zeros(3))

    def test_raises_velocity_wrong_len(self) -> None:
        with self.assertRaises(ValueError):
            self.body.set_velocity(np.zeros(3), np.zeros(2))
        with self.assertRaises(ValueError):
            self.body.set_velocity(np.zeros(2), np.zeros(3))

    def test_raises_joints_wrong_len(self) -> None:
        with self.assertRaises(AssertionError):
            self.body.configure_joints(np.zeros(1))


if __name__ == "__main__":
    absltest.main()
