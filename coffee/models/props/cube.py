from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts


class Cube(prop.TemplatedProp):
    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "cube",
        width: float = consts._BLOCK_WIDTH,
        height: float = consts._BLOCK_HEIGHT,
        depth: float = consts._BLOCK_DEPTH,
        mass: float = consts._BLOCK_MASS,
        color: Tuple[float, float, float, float] = consts._BLOCK_COLOR,
    ) -> None:
        replace_dict = {
            "WIDTH": width,
            "HEIGHT": height,
            "DEPTH": depth,
            "MASS": mass,
            "R": color[0],
            "G": color[1],
            "B": color[2],
            "A": color[3],
            "IXX": mass * (height ** 2 + depth ** 2) / 12,
            "IYY": mass * (width ** 2 + depth ** 2) / 12,
            "IZZ": mass * (width ** 2 + height ** 2) / 12,
        }
        filename = consts._PROP_ROOT / "cube" / "cube-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            body_id = pb_client.load_urdf(urdf)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)
