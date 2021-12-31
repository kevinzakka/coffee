import math
import time

import numpy as np
from dm_robotics.geometry import geometry, pose_distribution

from coffee import client
from coffee.manipulators.manipulator import Manipulator, ManipulatorConfig

_HOMEJ = [j * math.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]
_NUM_POSES = 10
_SEED = 42


def hang(bullet_client: client.BulletClient) -> None:
    try:
        while True:
            bullet_client.step()
            time.sleep(0.01)
    except KeyboardInterrupt:
        return


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
        max_joint_position_error=1e-2,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_link_name="ee_link",
        fixed_base=True,
    )
    # manipulator_config = ManipulatorConfig(
    #     urdf="robots/ufactory/xarm_description/xarm7.urdf",
    #     joint_resting_configuration=[0.0] * 7,
    #     max_joint_position_error=1e-2,
    #     max_joint_velocity_error=5e-2,
    #     movement_timeout=5.0,
    #     ik_point_link_name="ee_link",
    #     fixed_base=True,
    # )
    # manipulator_config = ManipulatorConfig(
    #     urdf="robots/franka_panda/panda.urdf",
    #     joint_resting_configuration=[0.98, 0.458, 0.34, -2.24, -0.30, 3.5, 2.32],
    #     max_joint_position_error=1e-2,
    #     max_joint_velocity_error=5e-2,
    #     movement_timeout=5.0,
    #     ik_point_link_name="ee_link",
    # )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )
        manipulator.go_home(disable_dynamics=True)
    manipulator.set_link_damping(0, 0)

    # Draw a coordinate frame centered at the manipulator's end effector.
    manipulator.debug_draw_ik_point_frame()

    # hang(bullet_client)

    pos_dist = pose_distribution.UniformPoseDistribution(
        min_pose_bounds=[0.20, -0.15, 0.20, 2 * np.pi / 3, -np.pi / 5, -np.pi / 4],
        max_pose_bounds=[0.25, +0.15, 0.30, 4 * np.pi / 3, np.pi / 5, np.pi / 4],
    )

    np.random.seed(_SEED)
    rng = np.random.RandomState(_SEED)

    for i in range(_NUM_POSES):
        ref_pose = geometry.Pose(*pos_dist.sample_pose(rng))
        manipulator.set_eef_pose(ref_pose, kp=3e-2, disable_dynamics=False)

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
