from typing import Tuple

from coffee import prop
from coffee.models.props import constants as consts


class Cube(prop.TemplatedProp):
    def _build(
        self,
        width: float = consts._BLOCK_WIDTH,
        height: float = consts._BLOCK_HEIGHT,
        depth: float = consts._BLOCK_DEPTH,
        mass: float = consts._BLOCK_MASS,
        color: Tuple[float, float, float, float] = consts._BLOCK_COLOR,
        *args,
        **kwargs,
    ) -> int:
        del args, kwargs

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
            return self.pb_client.load_urdf(urdf)
