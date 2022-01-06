from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts

_CUBE_ROOT = consts._PROP_ROOT / "cube"


class Cube(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        width: float = consts._BLOCK_WIDTH,
        height: float = consts._BLOCK_HEIGHT,
        depth: float = consts._BLOCK_DEPTH,
        mass: float = consts._BLOCK_MASS,
        color: Tuple[float, float, float, float] = consts._BLOCK_COLOR,
    ) -> None:
        super().__init__(name)

        self._width = width
        self._height = height
        self._depth = depth
        self._mass = mass
        self._color = color

        # Computed using the formula from Wikipedia:
        # https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
        self._ixx = mass * (height ** 2 + depth ** 2) / 12
        self._iyy = mass * (width ** 2 + depth ** 2) / 12
        self._izz = mass * (width ** 2 + height ** 2) / 12

    def _build(self, pb_client: BulletClient) -> None:
        replace_dict = {
            "WIDTH": self._width,
            "HEIGHT": self._height,
            "DEPTH": self._depth,
            "MASS": self._mass,
            "R": self._color[0],
            "G": self._color[1],
            "B": self._color[2],
            "A": self._color[3],
            "IXX": self._ixx,
            "IYY": self._iyy,
            "IZZ": self._izz,
        }
        filename = _CUBE_ROOT / "cube-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
