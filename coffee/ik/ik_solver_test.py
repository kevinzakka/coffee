"""Tests for ik_solver."""

import numpy as np
from absl.testing import absltest, parameterized
from dm_robotics.geometry import geometry, pose_distribution
from dm_robotics.transformations import transformations as tr

from coffee.ik.ik_solver import IKSolver
from coffee.joints import Joints
from coffee.utils.testing_utils import BulletMultiDirectParameterizedTestCase

# Linear and angular tolerance when comparing the end pose and the target pose.
_LINEAR_TOL = 1e-4
_ANGULAR_TOL = 1e-4

# How many test end-effector poses to generate.
_NUM_POSES = 50

# RNG seed.
_SEED = 42

# TODO(kevin): Remove this once we create robot specific configs and a robot factory
# class.
_ARM_PARAMS = [
    {
        "testcase_name": "arm_ur5",
        "urdf": "robot_arms/universal_robot/ur5_description/ur5.urdf",
        "joint_resting_configuration": np.array(
            [j * np.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]
        ),
        "base_position": np.zeros(3),
    },
    {
        "testcase_name": "arm_xarm7",
        "urdf": "robot_arms/ufactory/xarm7_description/xarm7.urdf",
        "joint_resting_configuration": np.array([0] * 7),
        "base_position": np.zeros(3),
    },
    {
        "testcase_name": "arm_iiwa",
        "urdf": "robot_arms/kuka/iiwa_description/urdf/iiwa14_polytope_collision.urdf",
        "joint_resting_configuration": np.array([0] * 7),
        # NOTE(kevin): The iiwa is a big robot, so we scooch it backwards a bit.
        "base_position": np.array([-0.1, 0, 0]),
    },
]


class IKSolverTest(BulletMultiDirectParameterizedTestCase):
    """Tests for IKSolver."""

    @parameterized.named_parameters(_ARM_PARAMS)
    def test_raises_when_nullspace_reference_wrong_length(
        self,
        urdf: str,
        joint_resting_configuration: np.ndarray,
        base_position: np.ndarray,
    ) -> None:
        body_id = self.client.load_urdf(urdf, pose=geometry.Pose(base_position))
        joints = Joints.from_body_id(body_id=body_id, pb_client=self.client)
        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=joints.get_joint_index_from_link_name("ee_link"),
            joints=joints,
            nullspace_reference=joint_resting_configuration,
        )
        ref_pose = geometry.Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        wrong_nullspace_ref = np.array([0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            ik_solver.solve(ref_pose, nullspace_reference=wrong_nullspace_ref)

    @parameterized.named_parameters(_ARM_PARAMS)
    def test_raises_when_initial_joint_confiugration_wrong_length(
        self,
        urdf: str,
        joint_resting_configuration: np.ndarray,
        base_position: np.ndarray,
    ) -> None:
        body_id = self.client.load_urdf(urdf, pose=geometry.Pose(base_position))
        joints = Joints.from_body_id(body_id=body_id, pb_client=self.client)
        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=joints.get_joint_index_from_link_name("ee_link"),
            joints=joints,
            nullspace_reference=joint_resting_configuration,
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
        urdf: str,
        joint_resting_configuration: np.ndarray,
        base_position: np.ndarray,
    ) -> None:
        # Seed to prevent flaky tests.
        np.random.seed(_SEED)
        rng = np.random.RandomState(_SEED)

        body_id = self.client.load_urdf(urdf, pose=geometry.Pose(base_position))
        joints = Joints.from_body_id(body_id=body_id, pb_client=self.client)

        ik_solver = IKSolver(
            pb_client=self.client,
            ik_point_joint_id=joints.get_joint_index_from_link_name("ee_link"),
            joints=joints,
            nullspace_reference=joint_resting_configuration,
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
                ref_pose, linear_tol=_LINEAR_TOL, angular_tol=_ANGULAR_TOL
            )
            self.assertIsNotNone(qpos_sol)
            assert qpos_sol is not None  # This is so mypy doesn't complain.
            min_range = joints.joints_lower_limit
            max_range = joints.joints_upper_limit
            np.testing.assert_array_compare(np.less_equal, qpos_sol, max_range)
            np.testing.assert_array_compare(np.greater_equal, qpos_sol, min_range)

            # Manually set the joints of the robot to the solution and perform FK.
            for i, joint_id in enumerate(joints.controllable_joints):
                self.client.resetJointState(
                    body_id,
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
