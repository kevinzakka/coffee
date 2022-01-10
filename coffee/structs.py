"""Constants and structs returned by the PyBullet API."""

import dataclasses
import enum
from typing import Optional, Tuple

# Constants.

BASE_LINK_ID: int = -1
STATIC_MASS: float = 0.0

# Structs.


class JointType(enum.Enum):
    """Enum for specifying the joint of type."""

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
class DynamicsInfo:
    """Holds the dynamics information of a link."""

    mass: float
    lateral_friction: float
    local_inertia_diagonal: Tuple[float, float, float]
    local_inertial_pos: Tuple[float, float, float]
    local_inertial_orn: Tuple[float, float, float, float]
    restitution: float
    rolling_friction: float
    spinning_friction: float
    contact_damping: float
    contact_stiffness: float
    body_type: int
    collision_margin: Optional[float] = None  # This one is unsupported.
