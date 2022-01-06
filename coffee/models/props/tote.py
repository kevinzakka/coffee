from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts

_TOTE_ROOT = consts._PROP_ROOT / "tote"


class Tote(prop.TemplatedProp):
    def __init__(
        self,
        name: str,
        scale: float = consts._TOTE_SCALE,
    ) -> None:
        super().__init__(name)

        self._mass = scale * consts._TOTE_BASE_MASS
        self._scale_x = scale * consts._TOTE_BASE_SX
        self._scale_y = scale * consts._TOTE_BASE_SY
        self._scale_z = scale * consts._TOTE_BASE_SZ

    def _build(self, pb_client: BulletClient) -> None:
        replace_dict = {
            "MASS": self._mass,
            "SX": self._scale_x,
            "SY": self._scale_y,
            "SZ": self._scale_z,
        }
        filename = _TOTE_ROOT / "tote-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            self._body_id = pb_client.load_urdf(urdf)
