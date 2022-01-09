"""Tests for camera."""

import numpy as np
import pybullet as p
from absl.testing import absltest
from dm_robotics.geometry import geometry

from coffee.sensors import CameraParams, Intrinsic
from coffee.sensors.camera import RPYCameraPose, URDFCameraPose
from coffee.utils.geometry_utils import as_quaternion_wxyz
from coffee.utils.testing_utils import BulletMultiDirectParameterizedTestCase


class CameraParamsTest(absltest.TestCase):
    def test_raises_when_width_or_height_less_than_one(self) -> None:
        with self.assertRaises(ValueError):
            CameraParams(width=0, height=1)
        with self.assertRaises(ValueError):
            CameraParams(width=1, height=0)

    def test_from_intrinsic(self) -> None:
        intr = Intrinsic(width=640, height=480, fx=450, fy=450, cx=320, cy=240)
        params = CameraParams.from_intrinsic(intr)
        self.assertAlmostEqual(params.fovy, 56.144973871705915)
        self.assertEqual(params.height, intr.height)
        self.assertEqual(params.width, intr.width)

    def test_properties(self) -> None:
        params = CameraParams()
        self.assertEqual(params.aspect_ratio, params.width / params.height)

    def test_znear_zfar_set(self) -> None:
        params = CameraParams(znear=0.01, zfar=10.0)
        new_params = params.set_znear(znear=1.0)
        self.assertEqual(params.znear, 0.01)
        self.assertEqual(new_params.znear, 1.0)
        newer_params = params.set_zfar(zfar=100.0)
        self.assertEqual(params.zfar, 10.0)
        self.assertEqual(newer_params.zfar, 100.0)


class RPYCameraPoseTest(absltest.TestCase):
    def test_from_rpy_init(self) -> None:
        rpy = np.random.uniform(0, 2 * np.pi, size=(3,))
        camera_pose = RPYCameraPose.from_posrpy(rpy=rpy)
        np.testing.assert_equal(camera_pose.pose.position, [0, 0, 0])
        camera_pose = RPYCameraPose.from_posrpy(pos=[5, 6, 7])
        np.testing.assert_equal(camera_pose.pose.quaternion, [1, 0, 0, 0])

    def test_from_rpy_matches_pybullet(self) -> None:
        rpy = np.random.uniform(0, 2 * np.pi, size=(3,))
        camera_pose = RPYCameraPose.from_posrpy(rpy=rpy)
        actual = camera_pose.pose
        quat_pybullet = p.getQuaternionFromEuler(rpy)
        expected = geometry.Pose(quaternion=as_quaternion_wxyz(quat_pybullet))
        self.assertTrue(expected == actual)


class URDFCameraPoseTest(BulletMultiDirectParameterizedTestCase):
    def setUp(self) -> None:
        super().setUp()

        self.body_id = self.client.load_urdf(
            "robot_arms/universal_robot/ur5_description/ur5_camera.urdf"
        )

    def test_raises_when_invalid_link_name(self) -> None:
        with self.assertRaises(ValueError):
            URDFCameraPose(
                pb_client=self.client,
                body_id=self.body_id,
                camera_link_name="invalid_link_name",
            )

    def test_from_urdf_init(self) -> None:
        # TODO(kevin): Make a better test where we can actually test the pose.
        URDFCameraPose(
            pb_client=self.client,
            body_id=self.body_id,
            camera_link_name="gripper_cam",
        )


if __name__ == "__main__":
    absltest.main()
