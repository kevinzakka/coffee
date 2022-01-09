import pathlib

import numpy as np

from coffee import _URDF_PATH

# Path to the URDF file.
XARM7_URDF: pathlib.Path = (
    _URDF_PATH / "robot_arms" / "ufactory" / "xarm7_description" / "xarm7.urdf"
)

# Degrees of freedom.
DOF = 7

# Resting joint configuration.
JOINT_RESTING_CONFIGURATION: np.ndarray = np.zeros(DOF)

# Last link name.
IK_POINT_LINK_NAME: str = "ee_link"

# Fixed base.
FIXED_BASE: bool = True

# Max joint position error.
MAX_JOINT_POSITION_ERROR: float = 1e-4
