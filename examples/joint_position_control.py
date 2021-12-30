import time

import numpy as np

from coffee import client
from coffee.client import BulletClient
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


def main() -> None:
    # Create the bullet client.
    bullet_client = BulletClient.create(
        client_config=client.ClientConfig(gui=True, realtime=True),
        physics_config=client.PhysicsConfig(),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Load plane.
    bullet_client.load_urdf("plane/plane.urdf", base_position=[0, 0, -0.001])

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        home_joint_configuration=[
            j * np.pi for j in [-1.0, -0.5, 0.5, -0.5, -0.5, 0.0]
        ],
        max_joint_position_error=1e-2,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_joint_name="wrist_3__fixed_joint__end_effector",
    )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )
        manipulator.go_home(disable_dynamics=True)

    time.sleep(1.0)  # Not needed.

    joint_positions = [
        [-3.1, -1.0, 1.6, -1.6, -1.6, 0.0],
        [-2.5, -1.0, 1.6, -1.6, -1.6, 0.0],
        [-2.5, -1.0, 1.6, -2.2, -1.6, 0.0],
        [-2.5, -1.6, 1.6, -2.2, -1.6, 0.0],
        [-3.1, -1.6, 1.6, -2.2, -1.6, 0.0],
        [-3.1, -1.6, 1.6, -1.6, -1.6, 0.0],
    ]
    for jp in joint_positions:
        manipulator.set_joint_positions(jp, blocking=True, speed=0.03)
        assert np.allclose(
            manipulator.get_joint_positions(),
            jp,
            atol=1e-2,
        )

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
