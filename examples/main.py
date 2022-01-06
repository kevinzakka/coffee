from coffee import client
from coffee.models import props


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        props.Tote("tote", bullet_client)
        p2 = props.Cube("block", bullet_client)
        p2.set_pose([0, 0, 0.1])
        p3 = props.Cylinder("cylinder", bullet_client, color=(0.3412, 0.3490, 1, 1))
        p3.set_pose([0.1, 0.0, 0.1])
        p4 = props.Cup("cup", bullet_client)
        p4.set_pose([0.1, 0.1, 0.1])
    bullet_client.infinite_step()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(realtime=True),
    )
    bullet_client.load_urdf("plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
