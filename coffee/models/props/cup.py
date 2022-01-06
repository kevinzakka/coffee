from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts

_CUP_ROOT = consts._PROP_ROOT / "cup"


class Cup(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        scale: float = consts._CUP_SCALE,
    ) -> None:
        super().__init__(name)

        self._mass = scale * consts._CUP_SCALE
        self._scale_x = scale * consts._CUP_BASE_SX
        self._scale_y = scale * consts._CUP_BASE_SY
        self._scale_z = scale * consts._CUP_BASE_SZ

    def _build(self, pb_client: BulletClient) -> None:
        replace_dict = {
            "MASS": self._mass,
            "SX": self._scale_x,
            "SY": self._scale_y,
            "SZ": self._scale_z,
        }
        filename = _CUP_ROOT / "cup-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
