import time

import numpy as np
from dm_robotics.geometry import geometry, pose_distribution

from coffee import client
from coffee.models import end_effectors, props, robot_arms
from coffee.robot import Robot
from coffee.wrappers import frame_visualizer


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        arm = robot_arms.UR5(bullet_client, enable_self_collision=True)
        # arm = robot_arms.xArm7(bullet_client)
        # arm = robot_arms.IIWA(bullet_client)
        # arm = robot_arms.Panda(bullet_client)

        gripper = end_effectors.robot_hands.SchunkWsg50(bullet_client, scaling=1.0)
        gripper.reset_pose([0.5, -0.5, 0.1])
        # gripper = end_effectors.robot_hands.RG2(bullet_client, scaling=1.0)
        # gripper.reset_pose([0.5, -0.5, 0.1])

        robot = Robot(
            pb_client=bullet_client,
            arm=arm,
            gripper=gripper,
            arm_effector=None,  # type: ignore
            gripper_effector=None,  # type: ignore
            name=f"{arm.name}_{gripper.name}",
        )
        robot.position_arm_joints(arm.joint_resting_configuration)

        block = props.Box(bullet_client, color=(0.3412, 0.3490, 1, 1))

    frame_visualizer.FrameVisualizer(robot, bullet_client).draw_link_frame(  # type: ignore
        robot.gripper.tool_center_point
    )

    # Create a pose distribution from which to sample the pose of the block.
    pos_dist = pose_distribution.UniformPoseDistribution(
        min_pose_bounds=[0.3, -0.5, 0.1, 0, 0, 0],
        max_pose_bounds=[0.55, +0.5, 0.1, 0, 0, np.pi],
    )
    rng = np.random.RandomState()

    for _ in range(5):
        # Randomize the object's pose.
        pos, quat = pos_dist.sample_pose(rng)
        block.reset_pose(pos, quat)
        steps_per_second = int(1 / bullet_client.config.physics_timestep)
        for _ in range(steps_per_second):
            bullet_client.step()

        pos, quat = block.get_pose()
        pos[-1] += block.half_extents[-1] + 5e-2  # Add 5cm slack.
        eef_pose = geometry.Pose(pos, quat)

        robot.position_gripper(eef_pose.position, eef_pose.quaternion)

        steps_per_second = int(0.5 / bullet_client.config.physics_timestep)
        for _ in range(steps_per_second):
            bullet_client.step()
            time.sleep(0.01)


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/short_floor.urdf")
    main(bullet_client)
    bullet_client.disconnect()
