from coffee import prop
from coffee.client import BulletClient
from coffee.models.props import constants as consts


class Tote(prop.TemplatedProp):
    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "tote",
        scale: float = consts._TOTE_SCALE,
    ) -> None:
        replace_dict = {
            "MASS": scale * consts._TOTE_BASE_MASS,
            "SX": scale * consts._TOTE_BASE_SX,
            "SY": scale * consts._TOTE_BASE_SY,
            "SZ": scale * consts._TOTE_BASE_SZ,
        }
        filename = consts._PROP_ROOT / "tote" / "tote-template.urdf"
        with self._fill_template(filename, replace_dict) as urdf:
            body_id = pb_client.load_urdf(urdf)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)
