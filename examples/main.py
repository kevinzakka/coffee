from coffee import client
from coffee.models.props import block


def main(bullet_client: client.BulletClient) -> None:
    # from ipdb import set_trace; set_trace()
    blk = block.Block(
        name="block",
        width=0.1,
        height=0.1,
        depth=0.1,
        color=(0.3412, 0.3490, 1, 1),
    )
    blk.build(bullet_client)
    bullet_client.infinite_step()

    # block.set_pose(bullet_client, position=(0.5, -0.1, 0.5), quaternion=(1, 0, 0, 0))


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(realtime=True),
    )
    bullet_client.load_urdf("plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
