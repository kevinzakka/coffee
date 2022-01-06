from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts


class Cylinder(prop.TemplatedProp):
    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "cylinder",
        radius: float = consts._CYLINDER_RADIUS,
        length: float = consts._CYLINDER_LENGTH,
        mass: float = consts._CYLINDER_MASS,
        color: Tuple[float, float, float, float] = consts._CYLINDER_COLOR,
    ) -> None:
        replace_dict = {
            "RADIUS": radius,
            "LENGTH": length,
            "MASS": mass,
            "R": color[0],
            "G": color[1],
            "B": color[2],
            "A": color[3],
            "IXX": mass * (3 * radius ** 2 + length ** 2) / 12,
            "IYY": mass * (3 * radius ** 2 + length ** 2) / 12,
            "IZZ": 0.5 * mass * radius ** 2,
        }
        filename = consts._PROP_ROOT / "cylinder" / "cylinder-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            body_id = pb_client.load_urdf(urdf)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)
