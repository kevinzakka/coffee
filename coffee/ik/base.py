from typing import Optional, Protocol

import numpy as np
from dm_robotics.geometry.geometry import Pose

from coffee.client import BulletClient
from coffee.joints import Joints


class IKSolverProtocol(Protocol):
    """Functionality that needs to be implemented by all inverse kinematics solvers."""

    pb_client: BulletClient
    joints: Joints
    ik_point_joint_id: int

    def solve(self, ref_pose: Pose) -> Optional[np.ndarray]:
        """Compute the IK solution given the pose of the IK target."""
        ...

    def forward_kinematics(self) -> Pose:
        """Compute the IK target pose given the current joint configuration."""
        ...
