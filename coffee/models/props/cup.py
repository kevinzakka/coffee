from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts


class Cup(prop.TemplatedProp):
    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "cup",
        mass: float = consts._CUP_BASE_MASS,
        scale: float = consts._CUP_SCALE,
    ) -> None:
        replace_dict = {
            "MASS": scale * mass,
            "SX": scale * consts._CUP_BASE_SX,
            "SY": scale * consts._CUP_BASE_SY,
            "SZ": scale * consts._CUP_BASE_SZ,
        }
        filename = consts._PROP_ROOT / "cup" / "cup-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            body_id = pb_client.load_urdf(urdf)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)
