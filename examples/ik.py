import time

import numpy as np
from dm_robotics.geometry import geometry, pose_distribution
from dm_robotics.transformations import transformations as tr

from coffee.ik.ik_solver import IKSolver, IKConfig
from coffee.joints import Joints
from coffee.client import BulletClient, ConnectionMode, ClientConfig

from coffee.cameras.camera_pybullet import (Intrinsic, PybulletCamera,
                                            PybulletCameraConfig,
                                            PybulletEndEffectorCamera)

# Linear and angular tolerance when comparing the end pose and the target pose.
_LINEAR_TOL = 1e-4
_ANGULAR_TOL = 1e-4

# How many test end-effector poses to generate.
_NUM_POSES = 10

_ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


def hang(bullet_client: BulletClient) -> None:
    try:
        while True:
            bullet_client.step_simulation()
            time.sleep(0.01)
    except KeyboardInterrupt:
        return


def main() -> None:
    server = BulletClient.create(
        mode=ConnectionMode.SHARED_MEMORY_SERVER,
        config=ClientConfig(realtime=True),
    )
    server.setAdditionalSearchPath(_ASSETS_PATH)

    # Seed to prevent flaky tests.
    np.random.seed(4379)
    rng = np.random.RandomState(4379)

    c1 = BulletClient.create(
        mode=ConnectionMode.SHARED_MEMORY,
        config=ClientConfig(realtime=True),
    )
    c1.setAdditionalSearchPath(_ASSETS_PATH)

    body_id = c1.load_urdf(
        "robots/universal_robot/ur_description/ur5_ravens.urdf",
        useFixedBase=True,
    )


    # Instantiate the camera.
    intr = Intrinsic(
        width=480,
        height=640,
        fx=450,
        fy=450,
        cx=320,
        cy=240,
    )
    camera_config = PybulletCameraConfig(
        image_size=(480, 640),
        intrinsics=intr,
        pose=SE3.from_rotation_and_translation(
            rotation=SO3(xyzw=front_rotation),
            translation=front_position,
        ),
        zrange=(0.01, 10.0),
    )
    static_camera = PybulletCamera(bullet_client, camera_config)


    # joints = Joints.from_body_id(
    #     body_id=body_id,
    #     pb_client=client,
    #     joint_resting_configuration=[
    #         j * np.pi for j in [0.0, 1.5, 0.5, 1.5, 1.5, 0.0]
    #     ],
    # )

    # ik_solver = BulletIKSolver(
    #     pb_client=client,
    #     body_id=body_id,
    #     ik_point_joint_id=joints.get_joint_index_from_link_name("ee_link"),
    #     joints=joints,
    #     ik_config=IKConfig(),
    # )

    # # Create a distribution from which reference poses will be sampled.
    # pos_dist = pose_distribution.UniformPoseDistribution(
    #     min_pose_bounds=[0.30, -0.15, 0.10, 2 * np.pi / 3, -np.pi / 5, -np.pi / 4],
    #     max_pose_bounds=[0.55, 0.15, 0.40, 4 * np.pi / 3, np.pi / 5, np.pi / 4],
    # )

    # # Each such iteration checks for the following:
    # #   1. Sample a reference pose from the distribution.
    # #   2. Solve IK for that pose to get a joint configuration.
    # #   3. Solve FK given the joint configuration.
    # #   4. Compare the FK pose to the reference pose.
    # for _ in range(_NUM_POSES):
    #     # Sample a reference pose.
    #     position, quaternion = pos_dist.sample_pose(rng)
    #     ref_pose = geometry.Pose(position, quaternion)

    #     # Check that we can solve the IK problem and that the solution respects the
    #     # joint limits.
    #     qpos_sol = ik_solver.inverse_kinematics(
    #         ref_pose,
    #         use_nullspace=False,
    #     )
    #     assert qpos_sol is not None
    #     min_range = joints.joints_lower_limit
    #     max_range = joints.joints_upper_limit
    #     np.testing.assert_array_compare(np.less_equal, qpos_sol, max_range)
    #     np.testing.assert_array_compare(np.greater_equal, qpos_sol, min_range)

    #     # Manually set the joints of the robot to the solution and perform FK.
    #     for i, joint_id in enumerate(joints.controllable_joints):
    #         client.resetJointState(
    #             body_id,
    #             joint_id,
    #             qpos_sol[i],
    #         )
    #     end_pose = ik_solver.forward_kinematics()

    #     # Check linear error is satisfied.
    #     linear_error = np.linalg.norm(end_pose.position - ref_pose.position)
    #     print(f"linear_error: {linear_error}")
    #     assert linear_error <= _LINEAR_TOL

    #     # Check angular error is satisfied.
    #     err_quat = tr.quat_diff_active(end_pose.quaternion, ref_pose.quaternion)
    #     err_axis_angle = tr.quat_to_axisangle(err_quat)
    #     angular_error = np.linalg.norm(err_axis_angle)
    #     print(f"angular_error: {angular_error}")
    #     assert angular_error <= _ANGULAR_TOL


if __name__ == "__main__":
    main()
