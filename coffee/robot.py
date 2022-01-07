"""A module for encapsulating the robot arm, bracelet and gripper."""

import abc

import numpy as np

from coffee.models.robot_arms import robot_arm


class Robot(abc.ABC):
    """Abstract base class for robotic arms and their attachments."""

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """The name of the robot."""

    @property
    @abc.abstractmethod
    def arm(self) -> robot_arm.RobotArm:
        """The arm."""

    @property
    @abc.abstractmethod
    def gripper(self):
        """The gripper."""

    @property
    @abc.abstractmethod
    def arm_base_site(self):
        """The site at the base of the arm."""

    @abc.abstractmethod
    def position_gripper(self, position: np.ndarray, quaternion: np.ndarray) -> None:
        """Moves the gripper IK point position to the desired pose."""

    @abc.abstractmethod
    def position_arm_joints(self, joint_angles: np.ndarray) -> None:
        """Moves the arm joints to the desired configuration."""


class StandardRobot(Robot):
    def __init__(
        self,
        arm: robot_arm.RobotArm,
        arm_base_site_name: str,
        gripper,
        arm_effector,
        gripper_effector,
        name: str = "robot",
    ) -> None:

        self._arm = arm
        self._gripper = gripper
        self._arm_base_site_name = arm_base_site_name
        self._arm_effector = arm_effector
        self._gripper_effector = gripper_effector
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    @property
    def arm(self) -> robot_arm.RobotArm:
        return self._arm

    @property
    def gripper(self):
        return self._gripper

    @property
    def arm_base_site(self):
        return self._arm_base_site_name

    def position_gripper(self, position: np.ndarray, quaternion: np.ndarray) -> None:
        pass

    def position_arm_joints(self, joint_angles: np.ndarray) -> None:
        self.arm.set_joint_angles(joint_angles)
