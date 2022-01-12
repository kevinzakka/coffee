"""Modify the PyBullet GUI, such as the color of the sky."""

import dataclasses
from typing import Optional, Tuple

import dcargs

from coffee import client


@dataclasses.dataclass(frozen=True)
class GuiArgs:
    render_shadows: bool = True
    color: Optional[Tuple[int, int, int]] = None
    width: Optional[int] = None
    height: Optional[int] = None


def main(args: GuiArgs) -> None:
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(**dataclasses.asdict(args)),
    )
    bullet_client.load_urdf("objects/plane/plane.urdf")
    bullet_client.infinite_step()
    bullet_client.disconnect()


if __name__ == "__main__":
    main(dcargs.parse(GuiArgs, description=__doc__))
