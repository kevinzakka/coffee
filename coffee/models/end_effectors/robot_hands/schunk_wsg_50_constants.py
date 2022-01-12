import pathlib

from coffee import _URDF_PATH

SCHUNK_WSG_50_URDF: pathlib.Path = (
    _URDF_PATH / "end_effectors" / "schunk" / "wsg_50_description" / "model.urdf"
)

# By default, Bullet disables self-collision.
ENABLE_SELF_COLLISION = False
