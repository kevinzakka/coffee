import pathlib

import numpy as np

from coffee import _SRC_ROOT

# Path to the URDF file.
UR5_URDF: pathlib.Path = (
    _SRC_ROOT / "models" / "vendor" / "universal_robot" / "ur_description" / "ur5.urdf"
)

# Resting joint configuration.
JOINT_RESTING_CONFIGURATION: np.ndarray = np.array(
    [j * np.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]
)

# Last link name.
IK_POINT_LINK_NAME: str = "ee_link"

# Fixed base.
FIXED_BASE: bool = True

# Max joint position error.
MAX_JOINT_POSITION_ERROR: float = 1e-4
