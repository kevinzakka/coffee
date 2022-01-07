import abc
from typing import Optional

import numpy as np

from coffee import body
from coffee.client import BulletClient
from coffee.hints import Array
from coffee.joints import JointState


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

    # @abc.abstractmethod
    def set_joint_angles(self, joint_angles: np.ndarray) -> None:
        """Sets the joints of the robot to a given configuration.

        Args:
            joint_angles: The desired joints configuration for the robot arm.
        """
        joint_angles = np.array(joint_angles, copy=False)
        assert len(joint_angles) == self.joints.dof

        # Clip the incoming joint angles to the actuation limits.
        joint_angles = np.clip(
            a=joint_angles,
            a_min=self.joints.joints_lower_limit,
            a_max=self.joints.joints_upper_limit,
        )

    # Methods to query the joint states.

    def get_joint_positions(self) -> np.ndarray:
        states = [
            JointState(*js)
            for js in self.pb_client.getJointStates(
                self.body_id, self.joints.controllable_joints
            )
        ]
        return np.asarray([state.joint_position for state in states])

    def get_joint_velocities(self) -> np.ndarray:
        states = [
            JointState(*js)
            for js in self.pb_client.getJointStates(
                self.body_id, self.joints.controllable_joints
            )
        ]
        return np.asarray([state.joint_velocity for state in states])

    def get_joint_torques(self) -> np.ndarray:
        states = [
            JointState(*js)
            for js in self.pb_client.getJointStates(
                self.body_id, self.joints.controllable_joints
            )
        ]
        return np.asarray([state.applied_joint_motor_torque for state in states])

    #

    def set_link_damping(self, linear: float, angular: float) -> None:
        """Sets the linear and angular damping of the robot arm's links.

        Args:
            linear (float): The linear damping value, between 0 and 1.
            angular (float): The angular damping value, between 0 and 1.
        """
        assert 0.0 <= linear <= 1.0
        assert 0.0 <= angular <= 1.0

        # NOTE(kevin): PyBullet's default is 0.04 for both.
        for j in range(self.pb_client.getNumJoints(self.body_id)):
            self.pb_client.changeDynamics(
                bodyUniqueId=self.body_id,
                linkIndex=j,
                linearDamping=linear,
                angularDamping=angular,
            )
