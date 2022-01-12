"""Place an arm in the world and reset its joints to their resting configuration."""

from coffee import client
from coffee.models import robot_arms


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        arm = robot_arms.UR5(bullet_client)
        arm.configure_joints(arm._joint_resting_configuration)
    bullet_client.infinite_step()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/short_floor.urdf")
    main(bullet_client)
    bullet_client.disconnect()
