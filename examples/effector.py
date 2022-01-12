"""Use a Cartesian 6D position effector to move the arm to a desired pose."""

import numpy as np
from dm_robotics.geometry import geometry, pose_distribution

from coffee import client
from coffee.effectors import cartesian_6d_position_effector
from coffee.models import props, robot_arms
from coffee.wrappers import FrameVisualizer


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        # Load the arm.
        arm = robot_arms.UR5(bullet_client)
        # arm = robot_arms.Panda(bullet_client)
        # arm = robot_arms.xArm7(bullet_client)
        # arm = robot_arms.IIWA(bullet_client)
        arm.configure_joints(arm._joint_resting_configuration)

    visualizer = FrameVisualizer(arm, bullet_client)
    visualizer.draw_link_frame(arm.ik_point_link_name)

    cartesian_effector = cartesian_6d_position_effector.Cartesian6dPositionEffector(
        model_params=cartesian_6d_position_effector.ModelParams(
            joints=arm.joints,
            link_id=arm.ik_point_link_id,
        ),
        control_params=cartesian_6d_position_effector.ControlParams(
            speed=1e-3,
            max_control_timesteps=5.0,
            max_joint_position_error=1e-3,
            nullspace_reference=arm.joint_resting_configuration,
        ),
        pb_client=bullet_client,
    )

    # Load a block prop and wait for it to settle.
    block = props.Box(bullet_client, color=(0.3412, 0.3490, 1, 1))
    steps_per_second = int(1 / bullet_client.config.physics_timestep)
    for _ in range(steps_per_second):
        bullet_client.step()

    # Create a pose distribution from which to sample the pose of the block.
    pos_dist = pose_distribution.UniformPoseDistribution(
        min_pose_bounds=[0.3, -0.5, 0.1, 0, 0, 0],
        max_pose_bounds=[0.55, +0.5, 0.1, 0, 0, 0],
    )
    rng = np.random.RandomState()

    for _ in range(5):
        # Randomize the object's pose.
        pos, quat = pos_dist.sample_pose(rng)
        block.reset_pose(pos, quat)
        steps_per_second = int(1 / bullet_client.config.physics_timestep)
        for _ in range(steps_per_second):
            bullet_client.step()

        pos, _ = block.get_pose()
        pos[-1] += block.half_extents[-1] + 1e-2  # Add 1cm slack.
        eef_pose = geometry.Pose(pos)

        cartesian_effector.set_control(eef_pose.to_posquat())


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/short_floor.urdf")
    main(bullet_client)
    bullet_client.disconnect()
