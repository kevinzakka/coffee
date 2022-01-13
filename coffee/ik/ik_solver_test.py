"""Tests for ik_solver."""

from typing import Type, Union

import numpy as np
from absl.testing import absltest, parameterized
from dm_robotics.geometry import geometry, pose_distribution
from dm_robotics.transformations import transformations as tr

from coffee.ik.ik_solver import IKSolver
from coffee.models import robot_arms
from coffee.utils.testing_utils import BulletMultiDirectParameterizedTestCase

RobotType = Union[robot_arms.UR5, robot_arms.Panda, robot_arms.xArm7, robot_arms.IIWA]

# Linear and angular tolerance when comparing the end pose and the target pose.
_LINEAR_TOL = 1e-4
_ANGULAR_TOL = 1e-4

# How many test end-effector poses to generate.
_NUM_POSES = 50

# RNG seed.
_SEED = 42

_ARM_PARAMS = [
    {
        "testcase_name": "arm_ur5",
        "robot_cls": robot_arms.UR5,
        "base_position": np.zeros(3),
    },
    {
        "testcase_name": "arm_panda",
        "robot_cls": robot_arms.Panda,
        "base_position": np.zeros(3),
    },
    {
        "testcase_name": "arm_xarm7",
        "robot_cls": robot_arms.xArm7,
        "base_position": np.zeros(3),
    },
    {
        "testcase_name": "arm_iiwa",
        "robot_cls": robot_arms.IIWA,
        # NOTE(kevin): The iiwa is a big robot, so we scooch it backwards a bit.
        "base_position": np.array([-0.1, 0, 0]),
    },
]


class IKSolverTest(BulletMultiDirectParameterizedTestCase):
    """Tests for IKSolver."""

    @parameterized.named_parameters(_ARM_PARAMS)
    def test_raises_when_nullspace_reference_wrong_length(
        self,
        robot_cls: Type[RobotType],
        base_position: np.ndarray,
    ) -> None:
        robot = robot_cls(pb_client=self.client)
        robot.reset_pose(base_position)
        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=robot.ik_point_link_id,
            joints=robot.joints,
            nullspace_reference=robot.joint_resting_configuration,
        )
        ref_pose = geometry.Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        wrong_nullspace_ref = np.array([0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            ik_solver.solve(ref_pose, nullspace_reference=wrong_nullspace_ref)

    @parameterized.named_parameters(_ARM_PARAMS)
    def test_raises_when_initial_joint_configuration_wrong_length(
        self,
        robot_cls: Type[RobotType],
        base_position: np.ndarray,
    ) -> None:
        robot = robot_cls(pb_client=self.client)
        robot.reset_pose(base_position)
        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=robot.ik_point_link_id,
            joints=robot.joints,
            nullspace_reference=robot.joint_resting_configuration,
        )
        ref_pose = geometry.Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        wrong_initial_joint_configuration = np.array([0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            ik_solver.solve(
                ref_pose, inital_joint_configuration=wrong_initial_joint_configuration
            )

    @parameterized.named_parameters(_ARM_PARAMS)
    def test_ik_solver_with_pose(
        self,
        robot_cls: Type[RobotType],
        base_position: np.ndarray,
    ) -> None:
        # Seed to prevent flaky tests.
        np.random.seed(_SEED)
        rng = np.random.RandomState(_SEED)

        robot = robot_cls(pb_client=self.client)
        robot.reset_pose(base_position)

        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=robot.ik_point_link_id,
            joints=robot.joints,
            nullspace_reference=robot.joint_resting_configuration,
        )

        # Create a distribution from which reference poses will be sampled.
        pos_dist = pose_distribution.UniformPoseDistribution(
            min_pose_bounds=[0.30, -0.25, 0.30, 2 * np.pi / 3, -np.pi / 5, -np.pi / 4],
            max_pose_bounds=[0.55, 0.25, 0.40, 4 * np.pi / 3, np.pi / 5, np.pi / 4],
        )

        # Each such iteration checks for the following:
        #   1. Sample a reference pose from the distribution.
        #   2. Solve IK for that pose to get a joint configuration.
        #   3. Solve FK given the joint configuration.
        #   4. Compare the FK pose to the reference pose.
        for _ in range(_NUM_POSES):
            # Sample a reference pose.
            ref_pose = geometry.Pose(*pos_dist.sample_pose(rng))

            # Check that we can solve the IK problem and that the solution respects the
            # joint limits.
            qpos_sol = ik_solver.solve(
                ref_pose,
                linear_tol=_LINEAR_TOL,
                angular_tol=_ANGULAR_TOL,
                num_attempts=100,
            )
            self.assertIsNotNone(qpos_sol)
            assert qpos_sol is not None  # This is so mypy doesn't complain.
            min_range = robot.joints.joints_lower_limit
            max_range = robot.joints.joints_upper_limit
            np.testing.assert_array_compare(np.less_equal, qpos_sol, max_range)
            np.testing.assert_array_compare(np.greater_equal, qpos_sol, min_range)

            # Manually set the joints of the robot to the solution and perform FK.
            for i, joint_id in enumerate(robot.joints.controllable_joints):
                self.client.resetJointState(
                    robot.body_id,
                    joint_id,
                    qpos_sol[i],
                )
            end_pose = ik_solver.forward_kinematics()

            # Check linear error is satisfied.
            linear_error = np.linalg.norm(end_pose.position - ref_pose.position)
            self.assertLessEqual(linear_error, _LINEAR_TOL)

            # Check angular error is satisfied.
            err_quat = tr.quat_diff_active(end_pose.quaternion, ref_pose.quaternion)
            err_axis_angle = tr.quat_to_axisangle(err_quat)
            angular_error = np.linalg.norm(err_axis_angle)
            self.assertLessEqual(angular_error, _ANGULAR_TOL)


if __name__ == "__main__":
    absltest.main()
