"""Tests for joints."""

import numpy as np
from absl.testing import absltest

from coffee.joints import Joints
from coffee.utils.testing_utils import _PI, BulletMultiDirectTestCase


class JointsTest(BulletMultiDirectTestCase):
    def test_ur5(self) -> None:
        body_id = self.client.load_urdf(
            "robot_arms/universal_robot/ur5_description/ur5.urdf"
        )
        joints = Joints.from_body_id(body_id, self.client)

        self.assertEqual(joints.dof, 6)
        self.assertTupleEqual(joints.controllable_joints, (0, 1, 2, 3, 4, 5))
        self.assertTupleEqual(joints.non_controllable_joints, (6,))

        self.assertLen(joints.joints_lower_limit, joints.dof)
        self.assertLen(joints.joints_upper_limit, joints.dof)
        self.assertLen(joints.joints_max_force, joints.dof)
        self.assertLen(joints.joints_max_velocity, joints.dof)
        self.assertLen(joints.zeros_array(), joints.dof)
        self.assertLen(joints.ones_array(), joints.dof)
        self.assertLen(joints.const_array(0), joints.dof)

        self.assertTrue(
            np.allclose(
                joints.joints_lower_limit,
                [-_PI, -_PI, -_PI, -_PI, -_PI, -_PI],
            )
        )

        self.assertTrue(
            np.allclose(
                joints.joints_upper_limit,
                [_PI, _PI, _PI, _PI, _PI, _PI],
            )
        )

        self.assertTrue(
            np.allclose(
                joints.joints_max_force,
                [150.0, 150.0, 150.0, 28.0, 28.0, 28.0],
            )
        )

        self.assertTrue(
            np.allclose(
                joints.joints_max_velocity,
                [3.15, 3.15, 3.15, 3.2, 3.2, 3.2],
            )
        )

        idx = joints.get_joint_index_from_joint_name("elbow_joint")
        self.assertEqual("elbow_joint", joints.get_joint_name_from_joint_index(idx))


if __name__ == "__main__":
    absltest.main()
