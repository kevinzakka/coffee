import math
import time

import imageio
import numpy as np

from coffee import client
from coffee.cameras.camera_pybullet import (Intrinsic, PybulletCamera,
                                            PybulletCameraConfig,
                                            PybulletEndEffectorCamera)
from coffee.lie import SE3, SO3
from coffee.manipulators.manipulator import (
    Manipulator, ManipulatorConfig)

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


def main() -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(gui=True, realtime=True),
        physics_config=client.PhysicsConfig(),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    # Load plane.
    bullet_client.load_urdf("plane/plane.urdf")

    front_position = (1.0, 0, 0.75)
    front_rotation = (np.pi / 4, np.pi, -np.pi / 2)
    front_rotation = bullet_client.getQuaternionFromEuler(front_rotation)

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5_camera.urdf",
        home_joint_configuration=[
            j * math.pi for j in [-1.0, -0.5, 0.5, -0.5, -0.5, 0.0]
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
    eef_camera = PybulletEndEffectorCamera(bullet_client, camera_config, manipulator)

    # Draw a coordinate frame centered at the manipulator's end effector.
    manipulator.debug_draw_ik_point_frame()

    bullet_client.load_urdf("block.urdf", base_position=[0.3, 0.0, 0.0], scaling=2.0)

    eef_frames = [eef_camera.get_frame().color]
    static_frames = [static_camera.get_frame().color]

    poses = [
        SE3.from_translation([0.3, 0.0, 0.2]),
        SE3.from_translation([0.3, 0.0, 0.1]),
        SE3.from_translation([0.3, 0.0, 0.08]),
    ]
    for pose in poses:
        print(f"Moving end-effector to: {pose}")
        manipulator.set_eef_pose(pose, speed=0.01, use_nullspace=False)
        eef_frames.append(eef_camera.get_frame().color)
        static_frames.append(static_camera.get_frame().color)
        time.sleep(0.1)

    # Concatenate the frames.
    frames = []
    for eef_frame, static_frame in zip(eef_frames, static_frames):
        frames.append(np.hstack([eef_frame, static_frame]))

    imageio.mimsave("/Users/kevin/repos/coffee/temp/render_camera.mp4", frames)

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
