import pathlib

import numpy as np

from coffee import _URDF_PATH

# Path to the URDF file.
UR5_URDF: pathlib.Path = (
    _URDF_PATH / "robot_arms" / "universal_robot" / "ur5_description" / "ur5.urdf"
)

# Degrees of freedom.
DOF = 6

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
