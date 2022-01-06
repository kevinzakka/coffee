from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts

_CYLINDER_ROOT = consts._PROP_ROOT / "cylinder"


class Cylinder(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        radius: float = consts._CYLINDER_RADIUS,
        length: float = consts._CYLINDER_LENGTH,
        mass: float = consts._CYLINDER_MASS,
        color: Tuple[float, float, float, float] = consts._CYLINDER_COLOR,
    ) -> None:
        super().__init__(name)

        self._radius = radius
        self._length = length
        self._mass = mass
        self._color = color

        # Computed using the formula from Wikipedia:
        # https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors
        self._ixx = mass * (3 * radius ** 2 + length ** 2) / 12
        self._iyy = self._ixx
        self._izz = 0.5 * mass * radius ** 2

    def _build(self, pb_client: BulletClient) -> None:
        replace_dict = {
            "RADIUS": self._radius,
            "LENGTH": self._length,
            "MASS": self._mass,
            "R": self._color[0],
            "G": self._color[1],
            "B": self._color[2],
            "A": self._color[3],
            "IXX": self._ixx,
            "IYY": self._iyy,
            "IZZ": self._izz,
        }
        filename = _CYLINDER_ROOT / "cylinder-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
