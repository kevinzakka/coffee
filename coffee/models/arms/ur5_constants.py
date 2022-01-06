import math

from coffee import _SRC_ROOT

# Home joint configuration.
HOMEJ = [j * math.pi for j in [0, -0.5, 0.5, -0.5, -0.5, 0.0]]

UR5_URDF = _SRC_ROOT / "models" / "vendor" / "universal_robot" / "ur5.urdf"
