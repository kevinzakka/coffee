from typing import Tuple

from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import _PROP_ROOT

_BLOCK_ROOT = _PROP_ROOT / "block"
RgbaColor = Tuple[float, float, float, float]


class Block(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        width: float = 0.04,
        height: float = 0.04,
        depth: float = 0.04,
        mass: float = 0.02,
        color: RgbaColor = (1, 0.3412, 0.3490, 1),
    ) -> None:
        super().__init__(name)

        self._width = width
        self._height = height
        self._depth = depth
        self._mass = mass
        self._color = color

    def _build(self, pb_client: BulletClient, *args, **kwargs) -> None:
        replace_dict = {
            "WIDTH": self._width,
            "HEIGHT": self._height,
            "DEPTH": self._depth,
            "MASS": self._mass,
            "R": self._color[0],
            "G": self._color[1],
            "B": self._color[2],
            "A": self._color[3],
        }
        filename = _BLOCK_ROOT / "block-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
