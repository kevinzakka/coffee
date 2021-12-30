import time

import numpy as np

from coffee.ik.ik_solver import IKConfig

np.set_printoptions(suppress=True, precision=7)
from coffee import client
from coffee.lie import SE3
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


def main() -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(
            gui=True, realtime=True, render_shadows=False
        ),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Load plane.
    bullet_client.load_urdf("plane/plane.urdf")

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/ufactory/xarm_description/xarm7.urdf",
        joint_resting_configuration=[0.0] * 7,
        max_joint_position_error=1e-4,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_link_name="ee_link",
        ik_config=IKConfig(joint_damping=0.0),
    )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )
        manipulator.go_home(disable_dynamics=True)

    # Draw a coordinate frame centered at the manipulator's end effector.
    manipulator.debug_draw_ik_point_frame()

    manipulator.set_link_damping(0.0, 0.0)

    bullet_client.load_urdf("block.urdf", base_position=[0.3, 0.0, 0.0], scaling=2.0)

    poses = [
        SE3.from_rotation_and_translation(
            translation=[0.3, 0.0, 0.4],
            rotation=manipulator.get_eef_pose().rotation,
        ),
        SE3.from_rotation_and_translation(
            translation=[0.3, 0.0, 0.2],
            rotation=manipulator.get_eef_pose().rotation,
        ),
        SE3.from_rotation_and_translation(
            translation=[0.3, 0.0, 0.081],
            rotation=manipulator.get_eef_pose().rotation,
        ),
    ]
    for pose in poses:
        manipulator.set_eef_pose(pose, kp=3e-2, use_nullspace=True)
        print(f"Commanded: {pose}")
        print(f"Actual: {manipulator.get_eef_pose()}")

    while True:
        bullet_client.step_simulation()
        time.sleep(0.01)

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
