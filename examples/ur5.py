from coffee import client
from coffee.models.robot_arms.ur5 import UR5


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        arm = UR5(bullet_client)
    arm.configure_joints(arm._joint_resting_configuration)
    bullet_client.infinite_step()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(realtime=True),
    )
    bullet_client.load_urdf("plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
