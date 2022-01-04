from __future__ import annotations

import dataclasses
import enum
from typing import Dict, Optional, Tuple

import numpy as np

from coffee.client import BulletClient
from coffee.hints import Array


class JointType(enum.Enum):
    """The type of joint."""

    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4


@dataclasses.dataclass(frozen=True)
class JointState:
    """Holds the state of a joint.

    Attributes:
        joint_position: The position of the joint.
        joint_velocity: The velocity of the joint.
        joint_reaction_forces: Joint reaction forces. If torque sensing is disabled for
            the joint, these will be all zeros.
        applied_joint_motor_torque: The motor torque applied during the last simulation
            step.
    """

    joint_position: float
    joint_velocity: float
    joint_reaction_forces: Tuple[float, ...]
    applied_joint_motor_torque: float


@dataclasses.dataclass(frozen=True)
class LinkState:
    """Holds the state of a link.

    Attributes:
        link_world_position: Position of the link COM in world coordinates.
        link_world_orientation: Orientation of the link COM in world coordinates.
        world_link_frame_position: Position of the URDF link frame in world coordinates.
        world_link_frame_orientation: Orientation of the URDF link frame in world
            coordinates.
    """

    link_world_position: Tuple[float, ...]
    link_world_orientation: Tuple[float, ...]
    local_inerital_frame_position: Tuple[float, ...]
    local_inerital_frame_orientation: Tuple[float, ...]
    world_link_frame_position: Tuple[float, ...]
    world_link_frame_orientation: Tuple[float, ...]
    world_link_linear_velocity: Optional[Tuple[float, ...]] = None
    world_link_angular_velocity: Optional[Tuple[float, ...]] = None


@dataclasses.dataclass(frozen=True)
class JointInfo:
    """Holds joint information."""

    joint_index: int
    joint_name: str
    joint_type: int
    q_index: int
    u_index: int
    flags: int
    joint_damping: float
    joint_friction: float
    joint_lower_limit: float
    joint_upper_limit: float
    joint_max_force: float
    joint_max_velocity: float
    link_name: str
    joint_axis: Tuple[float, ...]
    parent_frame_pos: Tuple[float, ...]
    parent_frame_orn: Tuple[float, ...]
    parent_index: int

    def __post_init__(self) -> None:
        super().__setattr__("joint_name", self.joint_name.decode("utf-8"))  # type: ignore
        super().__setattr__("link_name", self.link_name.decode("utf-8"))  # type: ignore


@dataclasses.dataclass(frozen=True)
class Joints:
    """A convenience class for accessing the joint information of a PyBullet body.

    These are parsed from the URDF. The extracted information is useful for things like
    inverse kinematics, which can take advantage of rest poses and joint limits to
    refine its solution.

    Attributes:
        body_id:
        joints_info:
        controllable_joints:
        non_controllable_joints:
        joint_resting_configuration:
    """

    body_id: int
    joints_info: Tuple[JointInfo, ...]
    controllable_joints: Tuple[int, ...]
    non_controllable_joints: Tuple[int, ...]
    joint_resting_configuration: Optional[np.ndarray]

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(bodyid={self.body_id}, dof={self.dof})"

    # Factory methods.

    @staticmethod
    def from_body_id(
        body_id: int,
        pb_client: BulletClient,
        joint_resting_configuration: Optional[Array] = None,
    ) -> Joints:
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

        if joint_resting_configuration is not None:
            if len(joint_resting_configuration) != len(controllable_joints):
                raise ValueError(
                    f"Resting configuration must have {len(controllable_joints)} elements."
                )
            joint_resting_configuration = np.array(joint_resting_configuration)

        return Joints(
            body_id=body_id,
            joints_info=tuple(joints_info),
            controllable_joints=tuple(controllable_joints),
            non_controllable_joints=tuple(non_controllable_joints),
            joint_resting_configuration=joint_resting_configuration,
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
        return np.asarray(lower, dtype=np.float64)

    @property
    def joints_upper_limit(self) -> np.ndarray:
        upper = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            if joint_info.q_index > -1:
                upper.append(joint_info.joint_upper_limit)
            else:
                upper.append(2.0 * np.pi)
        return np.asarray(upper, dtype=np.float64)

    @property
    def joints_range(self) -> np.ndarray:
        # Shape: (dof, 2).
        return np.vstack([self.joints_lower_limit, self.joints_upper_limit]).T

    @property
    def joints_max_force(self) -> np.ndarray:
        max_force = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            max_force.append(joint_info.joint_max_force)
        return np.asarray(max_force, dtype=np.float32)

    @property
    def joints_max_velocity(self) -> np.ndarray:
        max_velocity = []
        for joint_info in [self.joints_info[i] for i in self.controllable_joints]:
            max_velocity.append(joint_info.joint_max_velocity)
        return np.asarray(max_velocity, dtype=np.float64)

    # Array creation.

    def zeros_array(self) -> np.ndarray:
        return np.zeros(self.dof, dtype=np.float64)

    def ones_array(self) -> np.ndarray:
        return np.ones(self.dof, dtype=np.float64)

    def const_array(self, value: float) -> np.ndarray:
        return np.full(self.dof, value, dtype=np.float64)
