"""An example of how to modify the PyBullet GUI."""

import dataclasses
import time
from typing import Optional, Tuple

import dcargs

from coffee import client

_ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


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
    bullet_client.setAdditionalSearchPath(_ASSETS_PATH)

    bullet_client.load_urdf("plane/plane.urdf")

    try:
        while True:
            bullet_client.stepSimulation()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bullet_client.disconnect()


if __name__ == "__main__":
    main(dcargs.parse(GuiArgs, description=__doc__))
