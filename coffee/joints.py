from __future__ import annotations

import dataclasses
from typing import Dict, Tuple

import numpy as np

from coffee.client import BulletClient
from coffee.structs import JointInfo, JointType


@dataclasses.dataclass(frozen=True)
class Joints:
    """A convenience class for accessing the joint information of a PyBullet body.

    These are parsed from the URDF. The extracted information is useful for things like
    inverse kinematics, which can take advantage of rest poses and joint limits to
    refine its solution.

    Attributes:
        body_id: The unique ID of the body.
        joints_info: A tuple of `JointInfo` objects, one for each joint.
        controllable_joints: A tuple of indices designating the controllable joints of
            the body.
        non_controllable_joints: A tuple of indices designating the non-controllable
            joints of the body.
    """

    body_id: int
    joints_info: Tuple[JointInfo, ...]
    controllable_joints: Tuple[int, ...]
    non_controllable_joints: Tuple[int, ...]

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(bodyid={self.body_id}, dof={self.dof})"

    # Factory methods.

    @staticmethod
    def from_body_id(body_id: int, pb_client: BulletClient) -> Joints:
        controllable_joints = []
        non_controllable_joints = []
        joints_info = []
        for i in range(pb_client.getNumJoints(body_id)):
            joint_info = JointInfo(*pb_client.getJointInfo(body_id, i))
            if joint_info.joint_type != JointType.FIXED.value:
                controllable_joints.append(joint_info.joint_index)
            else:
                non_controllable_joints.append(joint_info.joint_index)
            joints_info.append(joint_info)

        return Joints(
            body_id=body_id,
            joints_info=tuple(joints_info),
            controllable_joints=tuple(controllable_joints),
            non_controllable_joints=tuple(non_controllable_joints),
        )

    # Accessors.

    def get_joint_index_from_joint_name(self, joint_name: str) -> int:
        for i, joint_info in enumerate(self.joints_info):
            if joint_info.joint_name == joint_name:
                return i
        raise ValueError(f"Joint {joint_name} not found.")

    def get_joint_name_from_joint_index(self, joint_index: int) -> str:
        return self.joints_info[joint_index].joint_name

    def get_joint_index_from_link_name(self, link_name: str) -> int:
        for i, joint_info in enumerate(self.joints_info):
            if joint_info.link_name == link_name:
                return i
        raise ValueError(f"Link {link_name} not found.")

    def get_link_name_from_joint_index(self, joint_index: int) -> str:
        return self.joints_info[joint_index].link_name

    def contains_link(self, link_name: str) -> bool:
        """Returns True if the given link name is present in the URDF."""
        for joint_info in self.joints_info:
            if joint_info.link_name == link_name:
                return True
        return False

    def contains_joint(self, joint_name: str) -> bool:
        """Returns True if the given joint name is present in the URDF."""
        for joint_info in self.joints_info:
            if joint_info.joint_name == joint_name:
                return True
        return False

    @property
    def link_names(self) -> Tuple[str, ...]:
        """Returns a tuple of link names."""
        return tuple(joint_info.link_name for joint_info in self.joints_info)

    @property
    def name2index(self) -> Dict[str, int]:
        """A dictionary mapping joint names to joint indices."""
        return {
            joint_info.joint_name: i for i, joint_info in enumerate(self.joints_info)
        }

    @property
    def index2name(self) -> Dict[int, str]:
        """A dictionary mapping joint indices to joint names."""
        return {
            i: joint_info.joint_name for i, joint_info in enumerate(self.joints_info)
        }

    @property
    def dof(self) -> int:
        return len(self.controllable_joints)

    @property
    def joints_lower_limit(self) -> np.ndarray:
        lower = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            if joint_info.q_index > -1:
                lower.append(joint_info.joint_lower_limit)
            else:
                lower.append(0.0)
        return np.array(lower, dtype=np.float64)

    @property
    def joints_upper_limit(self) -> np.ndarray:
        upper = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            if joint_info.q_index > -1:
                upper.append(joint_info.joint_upper_limit)
            else:
                upper.append(2.0 * np.pi)
        return np.array(upper, dtype=np.float64)

    @property
    def joints_range(self) -> np.ndarray:
        # Shape: (dof, 2).
        return np.vstack([self.joints_lower_limit, self.joints_upper_limit]).T

    @property
    def joints_max_force(self) -> np.ndarray:
        max_force = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            max_force.append(joint_info.joint_max_force)
        return np.array(max_force, dtype=np.float32)

    @property
    def joints_max_velocity(self) -> np.ndarray:
        max_velocity = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            max_velocity.append(joint_info.joint_max_velocity)
        return np.array(max_velocity, dtype=np.float64)

    # Array creation.

    def zeros_array(self) -> np.ndarray:
        return np.zeros(self.dof, dtype=np.float64)

    def ones_array(self) -> np.ndarray:
        return np.ones(self.dof, dtype=np.float64)

    def const_array(self, value: float) -> np.ndarray:
        return np.full(self.dof, value, dtype=np.float64)
