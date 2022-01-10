from coffee import client
from coffee.models import props


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        props.Tote(bullet_client)
        p2 = props.Box(bullet_client, color=(1, 0.3412, 0.3490, 1))
        p2.reset_pose([0, 0, 0.1])
        p3 = props.Cylinder(bullet_client, color=(0.3412, 0.3490, 1, 1))
        p3.reset_pose([0.1, 0.0, 0.1])
        p4 = props.Cup(bullet_client)
        p4.reset_pose([0.1, 0.1, 0.1])
        p5 = props.Sphere(bullet_client)
        p5.reset_pose([-0.1, 0.1, 0.1])
        p6 = props.Capsule(bullet_client)
        p6.reset_pose([0.0, -0.1, 0.1])
    bullet_client.infinite_step()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
