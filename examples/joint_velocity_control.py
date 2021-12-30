import dataclasses
import math
import time

import dcargs

from coffee import client
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


@dataclasses.dataclass
class Args:
    amplitude: float = 1.0
    frequency: float = 0.8


def sin_wave(
    time: float,
    frequency: float,
    amplitude: float,
) -> float:
    return amplitude * math.cos(2 * math.pi * frequency * time)


def main(args: Args) -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(gui=True, realtime=True),
        physics_config=client.PhysicsConfig(),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        home_joint_configuration=[0.0, -1.66, -1.92, -1.12, 1.57, 0.0],
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

    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        velocities = [
            sin_wave(elapsed_time, args.frequency, args.amplitude)
        ] * manipulator.joints.dof
        manipulator.set_joint_velocities(velocities)
        time.sleep(0.01)


if __name__ == "__main__":
    main(dcargs.parse(Args))
