from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts


class Ball(prop.TemplatedProp):
    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "ball",
        radius: float = consts._BALL_RADIUS,
        mass: float = consts._BALL_MASS,
        color: Tuple[float, float, float, float] = consts._BALL_COLOR,
    ) -> None:
        replace_dict = {
            "RADIUS": radius,
            "MASS": mass,
            "R": color[0],
            "G": color[1],
            "B": color[2],
            "A": color[3],
            "IXX": 2 * mass * (radius ** 2) / 5,
            "IYY": 2 * mass * (radius ** 2) / 5,
            "IZZ": 2 * mass * (radius ** 2) / 5,
        }
        filename = consts._PROP_ROOT / "ball" / "ball-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            body_id = pb_client.load_urdf(urdf)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)
