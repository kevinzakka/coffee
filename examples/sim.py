import math
import time

import numpy as np

np.set_printoptions(suppress=True, precision=3)

from coffee import client
from coffee.joints import Joints
from coffee.lie import SE3, SO3
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"

FPS = 30.0
PHYSICS_TIMESTEP = 1.0 / 240.0
PHYSICS_STEPS = 10
RENDER_TIMESTEP = 1.0 / 30
PHYSICS_TIMESTEP_NUM = RENDER_TIMESTEP / PHYSICS_TIMESTEP
assert PHYSICS_TIMESTEP_NUM.is_integer()
PHYSICS_TIMESTEP_NUM = int(PHYSICS_TIMESTEP_NUM)


def main() -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(
            gui=True,
            realtime=False,
            physics_timestep=PHYSICS_TIMESTEP,
        ),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Load plane.
    bullet_client.load_urdf("plane/plane.urdf", useMaximalCoordinates=True)

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        home_joint_configuration=[
            j * math.pi for j in [-1.0, -0.5, 0.5, -0.5, -0.5, 0.0]
        ],
        max_joint_position_error=1e-3,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_link_name="ee_link",
        fixed_base=True,
    )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )
        manipulator.go_home(disable_dynamics=True)

    # Draw a coordinate frame centered at the manipulator's end effector.
    manipulator.debug_draw_ik_point_frame()

    bullet_client.load_urdf("block.urdf", base_position=[0.3, 0.0, 0.2], scaling=2.0)

    spf = 1.0 / FPS
    dt = spf / PHYSICS_STEPS
    print(dt)
    for i in range(PHYSICS_TIMESTEP_NUM):
        bullet_client.step_simulation()
        time.sleep(dt)

    # manipulator.set_joint_positions([0.0] * 6)
    # for _ in range(PHYSICS_TIMESTEP_NUM):
    #     bullet_client.step_simulation()

    print("================================================")
    # while True:
    #     bullet_client.step_simulation()
    #     time.sleep(0.01)

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
