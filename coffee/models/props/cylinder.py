from typing import Tuple

from coffee import prop
from coffee.models.props import constants as consts


class Cylinder(prop.TemplatedProp):
    def _build(
        self,
        radius: float = consts._CYLINDER_RADIUS,
        length: float = consts._CYLINDER_LENGTH,
        mass: float = consts._CYLINDER_MASS,
        color: Tuple[float, float, float, float] = consts._CYLINDER_COLOR,
        *args,
        **kwargs,
    ) -> int:
        del args, kwargs

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
            return self.pb_client.load_urdf(urdf)
