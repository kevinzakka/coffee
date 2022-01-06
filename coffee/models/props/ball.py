from typing import Tuple

from coffee import prop
from coffee.models.props import constants as consts


class Ball(prop.TemplatedProp):
    def _build(
        self,
        radius: float = consts._BALL_RADIUS,
        mass: float = consts._BALL_MASS,
        color: Tuple[float, float, float, float] = consts._BALL_COLOR,
        *args,
        **kwargs,
    ) -> int:
        del args, kwargs

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
            return self.pb_client.load_urdf(urdf)
