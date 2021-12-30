"""Tests position control tracking."""

import math
from dataclasses import dataclass

import dcargs
import matplotlib.pyplot as plt
import numpy as np

from coffee import client
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


@dataclass
class Args:
    num_test_samples: int = 5
    max_joint_position_error: float = 1e-3
    speed: float = 3e-2


def main(args: Args) -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(gui=False, realtime=True),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        home_joint_configuration=[
            j * math.pi for j in [-1.0, -0.5, 0.5, -0.5, -0.5, 0.0]
        ],
        max_joint_position_error=args.max_joint_position_error,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_joint_name="wrist_3__fixed_joint__end_effector",
    )
    with bullet_client.disable_rendering():
        manipulator = Manipulator.create(
            manipulator_config=manipulator_config,
            pb_client=bullet_client,
        )

    expected_configurations = [
        np.random.uniform(
            low=manipulator.joints.joints_lower_limit,
            high=manipulator.joints.joints_upper_limit,
        )
        for _ in range(args.num_test_samples)
    ]
    actual_configurations = []
    for i, expected_configuration in enumerate(expected_configurations):
        print(f"({i+1}/{args.num_test_samples}) moving to: {expected_configuration}")

        manipulator.set_joint_positions(expected_configuration, speed=args.speed)
        actual_configuration = manipulator.get_joint_positions()
        actual_configurations.append(actual_configuration)

        diff = actual_configuration - expected_configuration
        if np.all(np.abs(diff) > manipulator_config.max_joint_position_error):
            raise RuntimeError("Joint position tracking failed.")

    fig, axes = plt.subplots(manipulator.joints.dof, 1)
    for i in range(manipulator.joints.dof):
        axes[i].plot([config[i] for config in actual_configurations], label="actual")
        axes[i].plot(
            [config[i] for config in expected_configurations], label="expected"
        )
    fig.tight_layout()
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main(dcargs.parse(Args))
