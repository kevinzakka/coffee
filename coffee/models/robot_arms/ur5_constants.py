# import enum
# import math

# from coffee import _SRC_ROOT


# # Available actuation methods available for the UR5.
# class Actuation(enum.Enum):
#     POSITION = 0
#     """Position control."""


# # Degrees of freedom. This is going to be checked against what is parsed from the
# # URDF file.
# DOF = 6

# # Home joint configuration.
# HOMEJ = [j * math.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]

# UR5_URDF = _SRC_ROOT / "models" / "vendor" / "universal_robot" / "ur5.urdf"


# DEFAULT_Kp: float = 3e-2
# DEFAULT_Kv: float = 0.0
