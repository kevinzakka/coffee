from coffee import prop
from coffee.models.props import constants as consts


class Cup(prop.TemplatedProp):
    def _build(
        self,
        scale: float = consts._CUP_SCALE,
        *args,
        **kwargs,
    ) -> int:
        del args, kwargs

        replace_dict = {
            "MASS": scale * consts._CUP_SCALE,
            "SX": scale * consts._CUP_BASE_SX,
            "SY": scale * consts._CUP_BASE_SY,
            "SZ": scale * consts._CUP_BASE_SZ,
        }
        filename = consts._PROP_ROOT / "cup" / "cup-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            return self.pb_client.load_urdf(urdf)
