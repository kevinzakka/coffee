import abc
from typing import Optional

import numpy as np

from coffee import body
from coffee.client import BulletClient
from coffee.hints import Array


class RobotArm(abc.ABC, body.NamedBody):
    """Robot arm base class."""

    @abc.abstractmethod
    def __init__(
        self,
        name: str,
        body_id: int,
        pb_client: BulletClient,
        joint_resting_configuration: Array,
        ik_point_link_name: Optional[str],
        fixed_base: bool,
        max_joint_position_error: float,
    ) -> None:
        """Constructor to be overridden by subclasses.

        Args:
            name: The unique name of the robot arm.
            body_id: The unique body id of the robot arm.
            pb_client: The BulletClient instance.
            joint_resting_configuration: The configuration of the joints at rest. Useful
                for biasing the solution space of the inverse kinematics solver.
            ik_point_link_name: The name of the link that will serve as the inverse
                kinematics target. If None, the link attached to the last controllable
                joint will be used.
            fixed_base: If True, forces the base of the manipulator to be static.
            max_joint_position_error: Threshold for joint position error.
        """
        super().__init__(name=name, body_id=body_id, pb_client=pb_client)

        self._joint_resting_configuration = joint_resting_configuration
        self._ik_point_link_name = ik_point_link_name
        self._fixed_base = fixed_base
        self._max_joint_position_error = max_joint_position_error

    @abc.abstractmethod
    def set_joint_angles(self, joint_angles: np.ndarray) -> None:
        """Sets the joints of the robot to a given configuration.

        Args:
            joint_angles: The desired joints configuration for the robot arm.
        """
