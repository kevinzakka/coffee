import matplotlib.pyplot as plt
import numpy as np
from dm_robotics.geometry import geometry

from coffee import client, sensors
from coffee.models.arms.old_robot_arm import Arm, ArmConfig
from coffee.sensors import camera


def main() -> None:
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.DIRECT,
        config=client.ClientConfig(realtime=True, render_shadows=True),
    )

    bullet_client.load_urdf("plane/plane.urdf")

    # Instantiate the manipulator.
    manipulator_config = ArmConfig(
        urdf="robots/universal_robot/ur_description/ur5.urdf",
        joint_resting_configuration=[
            j * np.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]
        ],
        max_joint_position_error=1e-4,
        max_joint_velocity_error=5e-2,
        movement_timeout=5.0,
        ik_point_link_name="ee_link",
        fixed_base=True,
    )
    manipulator = Arm.create(
        arm_config=manipulator_config,
        pb_client=bullet_client,
    )

    # Projection parameters.
    camera_params = camera.CameraParams(width=640, height=480, fovy=60.0)
    print("projection matrix: ", camera_params.as_projection_matrix())

    # Viewing parameters
    pose = geometry.Pose.from_poseuler([0.0, -0.1, 0.05, 0, 0, 0])

    cam_pose = camera.RelativeToCameraPose(
        pb_client=bullet_client,
        body_id=manipulator.body_id,
        relative_pose=pose,
        link_name="ee_link",
    )
    print("view matrix: ", cam_pose.as_view_matrix())

    cam = sensors.RelativeToCamera(bullet_client, camera_params, cam_pose)

    plt.imshow(cam.render().color)
    plt.show()

    cube_id = bullet_client.load_urdf(
        "block.urdf",
        scaling=2.0,
        pose=geometry.Pose([0.6, 0, 0]),
        useFixedBase=True,
    )
    for _ in range(int(2 / bullet_client.config.physics_timestep)):
        bullet_client.step()

    manipulator.go_home(disable_dynamics=True)

    plt.imshow(cam.render().color)
    plt.show()

    eef_pose = manipulator.get_eef_pose()
    cube_pose = bullet_client.get_body_state(cube_id).pose
    delta = 0.045
    pose = geometry.Pose(
        np.array(cube_pose.position) + np.array([0.0, 0.0, delta]),
        eef_pose.quaternion,
    )
    manipulator.set_eef_pose(pose, kp=3e-2)
    plt.imshow(cam.render().color)
    plt.show()

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
