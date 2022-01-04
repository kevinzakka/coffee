import math
import time
from typing import List

import numpy as np
from dm_robotics.geometry import geometry, pose_distribution

from coffee import client
from coffee.manipulators.manipulator import Manipulator, ManipulatorConfig

_HOMEJ = [j * math.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]
_CUBE_THICKNESS = 0.045
_CUBE_SCALE = 2.0


def get_eef_trajectory(
    current_pose: geometry.Pose,
    bullet_client: client.BulletClient,
    cube_id: int,
) -> List[geometry.Pose]:
    ret = []
    cube_pose = bullet_client.get_body_state(cube_id).pose
    delta = _CUBE_SCALE * _CUBE_THICKNESS / 2
    ret.append(  # pre-grasp
        geometry.Pose(
            np.array(cube_pose.position) + np.array([0.0, 0.0, delta + 0.1]),
            current_pose.quaternion,
        )
    )
    ret.append(  # grasp
        geometry.Pose(
            np.array(cube_pose.position) + np.array([0.0, 0.0, delta]),
            current_pose.quaternion,
        )
    )
    ret.append(  # post-grasp
        geometry.Pose(
            np.array(cube_pose.position) + np.array([0.0, 0.0, delta + 0.1]),
            current_pose.quaternion,
        )
    )
    return ret


def main() -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(realtime=True, render_shadows=False),
    )

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        joint_resting_configuration=_HOMEJ,
        max_joint_position_error=1e-4,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_link_name="ee_link",
        fixed_base=True,
    )
    # manipulator_config = ManipulatorConfig(
    #     urdf="robots/ufactory/xarm_description/xarm7.urdf",
    #     joint_resting_configuration=[0.0] * 7,
    #     max_joint_position_error=1e-3,
    #     max_joint_velocity_error=5e-2,
    #     movement_timeout=5.0,
    #     ik_point_link_name="ee_link",
    #     fixed_base=True,
    # )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )
        manipulator.go_home(disable_dynamics=True)

    # Draw a coordinate frame centered at the manipulator's end effector.
    manipulator.debug_draw_ik_point_frame()

    # Disable link damping.
    manipulator.set_link_damping(0, 0)

    # Load the cube.
    cube_id = bullet_client.load_urdf(
        "block.urdf",
        scaling=_CUBE_SCALE,
        pose=geometry.Pose([0.4, 0, 0]),
        useFixedBase=True,
    )

    bullet_client.infinite_step()

    pos_dist = pose_distribution.UniformPoseDistribution(
        min_pose_bounds=[-0.5, -0.5, 0.0, 0, 0, 0],
        max_pose_bounds=[0.5, +0.5, 0.0, 0, 0, 0],
    )

    bullet_client.infinite_step()

    np.random.seed(42)
    rng = np.random.RandomState(42)

    for _ in range(50):
        pose = geometry.Pose(*pos_dist.sample_pose(rng))
        bullet_client.reset_body_state(cube_id, pose)
        for _ in range(50):
            bullet_client.step()
            time.sleep(0.01)
        eef_poses = get_eef_trajectory(
            manipulator.get_eef_pose(),
            bullet_client,
            cube_id,
        )
        for eef_pose in eef_poses:
            manipulator.set_eef_pose(eef_pose, kp=3e-2)
        manipulator.go_home()

    bullet_client.disconnect()


if __name__ == "__main__":
    main()