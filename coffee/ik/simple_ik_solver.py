from __future__ import annotations

import dataclasses
from typing import Optional

import numpy as np
import pybullet as p
from dm_robotics.geometry.geometry import Pose

from coffee.client import BulletClient
from coffee.joints import Joints
from coffee.structs import LinkState
from coffee.utils import geometry_utils


@dataclasses.dataclass
class SimpleIKSolver:
    """A simple IK solver.

    This is a barebones wrapper over PyBullet's `calculateInverseKinematics` method. It
    does not use any of the nullspace arguments in the function call. For a more robust
    IK solver, see `ik_solver.IKSolver`.
    """

    pb_client: BulletClient
    joints: Joints
    ik_point_joint_id: int
    joint_damping: float = 1e-5

    def solve(
        self,
        ref_pose: Pose,
        max_num_iterations: int = 500,
        residual_threshold: float = 1e-5,
    ) -> Optional[np.ndarray]:
        try:
            qpos = self.pb_client.calculateInverseKinematics(
                bodyUniqueId=self.joints.body_id,
                endEffectorLinkIndex=self.ik_point_joint_id,
                targetPosition=ref_pose.position,
                targetOrientation=geometry_utils.as_quaternion_xyzw(
                    ref_pose.quaternion
                ),
                residualThreshold=residual_threshold,
                maxNumIterations=max_num_iterations,
                jointDamping=self.joints.const_array(self.joint_damping).tolist(),
            )
            if np.isnan(np.sum(qpos)):
                qpos = None
            else:
                # Clip to joint limits.
                qpos = np.clip(
                    a=np.array(qpos),
                    a_min=self.joints.joints_lower_limit,
                    a_max=self.joints.joints_upper_limit,
                )
        except p.error:
            qpos = None
        return qpos

    def forward_kinematics(self) -> Pose:
        eef_link_state = LinkState(
            *self.pb_client.getLinkState(
                bodyUniqueId=self.joints.body_id,
                linkIndex=self.ik_point_joint_id,
                computeLinkVelocity=0,
                computeForwardKinematics=True,
            )
        )

        return Pose(
            position=eef_link_state.link_world_position,
            quaternion=geometry_utils.as_quaternion_wxyz(
                eef_link_state.link_world_orientation
            ),
        )
