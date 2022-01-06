from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts

_BALL_ROOT = consts._PROP_ROOT / "ball"


class Ball(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        radius: float = consts._BALL_RADIUS,
        mass: float = consts._BALL_MASS,
        color: Tuple[float, float, float, float] = consts._BALL_COLOR,
    ) -> None:
        super().__init__(name)

        self._radius = radius
        self._mass = mass
        self._color = color

        # Computed using the formula from Wikipedia:
        # https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
        self._ixx = 2 * mass * (radius ** 2) / 5
        self._iyy = self._ixx
        self._izz = self._ixx

    def _build(self, pb_client: BulletClient) -> None:
        replace_dict = {
            "RADIUS": self._radius,
            "MASS": self._mass,
            "R": self._color[0],
            "G": self._color[1],
            "B": self._color[2],
            "A": self._color[3],
            "IXX": self._ixx,
            "IYY": self._iyy,
            "IZZ": self._izz,
        }
        filename = _BALL_ROOT / "ball-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
