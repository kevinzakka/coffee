import pathlib

from coffee import _URDF_PATH

RG2_URDF: pathlib.Path = (
    _URDF_PATH / "end_effectors" / "onrobot" / "rg2_description" / "model.urdf"
)

# By default, Bullet disables self-collision.
ENABLE_SELF_COLLISION = False
