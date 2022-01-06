from coffee import prop
from coffee.models.props import constants as consts


class Tote(prop.TemplatedProp):
    def _build(self, scale: float = consts._TOTE_SCALE, *args, **kwargs) -> int:
        del args, kwargs

        replace_dict = {
            "MASS": scale * consts._TOTE_BASE_MASS,
            "SX": scale * consts._TOTE_BASE_SX,
            "SY": scale * consts._TOTE_BASE_SY,
            "SZ": scale * consts._TOTE_BASE_SZ,
        }
        filename = consts._PROP_ROOT / "tote" / "tote-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            return self.pb_client.load_urdf(urdf)
