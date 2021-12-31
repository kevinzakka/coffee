import math
import time

import imageio
import numpy as np
from dm_robotics.geometry import geometry
from dm_robotics.transformations import transformations as tr

from coffee import cameras, client
from coffee.manipulators.manipulator import Manipulator, ManipulatorConfig

_ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"
_HOMEJ = [j * math.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]


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
    bullet_client.setAdditionalSearchPath(_ASSETS_PATH)

    # Load plane.
    bullet_client.load_urdf("plane/plane.urdf")

    # Instantiate the manipulator.
    manipulator_config = ManipulatorConfig(
        urdf="robots/universal_robot/ur_description/ur5_camera.urdf",
        joint_resting_configuration=_HOMEJ,
        max_joint_position_error=1e-4,
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

    # Draw a coordinate frame at the camera joint.
    manipulator.debug_draw_frame("gripper_cam")

    bullet_client.infinite_step()

    # Intrinsics matrix.
    intr = cameras.Intrinsic(480, 640, 450, 450, 320, 240)

    # Extrinsics.
    extr = geometry.Pose(
        position=[1.0, 0.0, 0.75],
        quaternion=tr.euler_to_quat([np.pi / 4, np.pi, -np.pi / 2], "XYZ"),
    )

    # Instantiate the camera.
    camera_config = cameras.CameraConfig(
        image_size=(480, 640),
        intrinsics=intr,
        pose=extr,
        zrange=(0.01, 10.0),
    )
    static_camera = cameras.Camera(bullet_client, camera_config)
    eef_camera = cameras.EndEffectorCamera(bullet_client, camera_config, manipulator)

    bullet_client.load_urdf("block.urdf", geometry.Pose([0.3, 0.0, 0.0]), scaling=2.0)

    eef_frames = [eef_camera.get_frame().color]
    static_frames = [static_camera.get_frame().color]

    poses = [
        geometry.Pose([0.3, 0.0, 0.2], manipulator.get_eef_pose().quaternion),
        geometry.Pose([0.3, 0.0, 0.1], manipulator.get_eef_pose().quaternion),
        geometry.Pose([0.3, 0.0, 0.08], manipulator.get_eef_pose().quaternion),
    ]
    for pose in poses:
        print(f"Moving end-effector to: {pose}")
        manipulator.set_eef_pose(pose)
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
