import matplotlib.pyplot as plt
import numpy as np
from dm_robotics.geometry import geometry

from coffee import client, sensors
from coffee.models import props, robot_arms
from coffee.sensors import camera


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        arm = robot_arms.UR5(bullet_client)
        arm.configure_joints(arm._joint_resting_configuration)

        block = props.Box(bullet_client, color=(0.3412, 0.3490, 1, 1))
        block.reset_pose([0.4, 0, 0.1])

    camera_params = camera.CameraParams(width=640, height=480, fovy=60.0)

    poses = [
        geometry.Pose.from_poseuler([0.0, -0.12, 0.1, -np.pi / 3, 0, 0]),
        geometry.Pose.from_poseuler([0.0, +0.12, 0.1, np.pi / 5, 0, np.pi]),
        geometry.Pose.from_poseuler([0.06, 0, 0.03, 0, -np.pi / 5, np.pi / 2]),
    ]

    views = []
    for pose in poses:
        cam_pose = camera.RelativeToCameraPose(
            pb_client=bullet_client,
            body_id=arm.body_id,
            relative_pose=pose,
            link_name=arm.ik_point_link_name,
        )
        cam = sensors.RelativeToCamera(bullet_client, camera_params, cam_pose)
        views.append(cam.render().color)

    _, axes = plt.subplots(1, len(poses))
    for ax, im in zip(axes, views):
        ax.imshow(im)
        ax.axis("off")
    plt.show()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.DIRECT,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
