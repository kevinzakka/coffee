import abc
import tempfile
from typing import Optional

import numpy as np

from coffee import body
from coffee.client import BulletClient
from coffee.hints import Array
from coffee.joints import Joints
from coffee.models.end_effectors.robot_hands import robot_hand
from coffee.third_party.urdf_editor import UrdfEditor


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

        self._joint_resting_configuration = np.array(joint_resting_configuration)
        self._ik_point_link_name = ik_point_link_name
        self._fixed_base = fixed_base
        self._max_joint_position_error = max_joint_position_error

    @property
    def joint_resting_configuration(self) -> np.ndarray:
        return self._joint_resting_configuration

    @property
    def ik_point_link_name(self) -> str:
        if self._ik_point_link_name is None:
            return self._joints.joints_info[-1].link_name
        return self._ik_point_link_name

    @property
    def ik_point_link_id(self) -> int:
        if self._ik_point_link_name is None:
            return self.joints.controllable_joints[-1]
        return self.joints.get_joint_index_from_link_name(self._ik_point_link_name)

    @abc.abstractmethod
    def set_joint_angles(self, joint_angles: np.ndarray) -> None:
        """Sets the joints of the robot to a given configuration.

        Args:
            joint_angles: The desired joints configuration for the robot arm.
        """

    def attach(
        self,
        hand: robot_hand.RobotHand,
    ) -> None:
        """Attaches a robot hand to the arm.

        PyBullet's `createConstraint` mechanism is broken. This function creates
        a new temporary URDF file by joining that of the arm and the hand, loads it
        into simulation and deletes the original arm and hand bodies.
        """
        with self._pb_client.disable_rendering():
            arm_editor = UrdfEditor()
            arm_editor.initializeFromBulletBody(self.body_id, self._pb_client.client_id)

            hand_editor = UrdfEditor()
            hand_editor.initializeFromBulletBody(
                hand.body_id, self._pb_client.client_id
            )

            newjoint = arm_editor.joinUrdf(
                childEditor=hand_editor,
                parentLinkIndex=arm_editor.linkNameToIndex[self._ik_point_link_name],
                jointPivotXYZInParent=[0, 0, 0],
                jointPivotRPYInParent=[np.pi, 0, 0],
                jointPivotXYZInChild=[0, 0, 0],
                jointPivotRPYInChild=[0, 0, 0],
                parentPhysicsClientId=self._pb_client.client_id,
                childPhysicsClientId=self._pb_client.client_id,
            )
            newjoint.joint_type = self._pb_client.JOINT_FIXED
            newjoint.joint_name = "joint_arm_gripper"

            self._tf = tempfile.NamedTemporaryFile(suffix=".urdf", delete=True)
            arm_editor.saveUrdf(self._tf.name)
            self._tf.flush()

            self._pb_client.remove_body(self._body_id)
            self._pb_client.remove_body(hand.body_id)

            self._body_id = self._pb_client.load_urdf(
                filename=self._tf.name,
                fixed_base=self._fixed_base,
            )

            self.configure_joints(self._joint_resting_configuration)

            # Reload the joint information.
            self._joints = Joints.from_body_id(
                body_id=self._body_id,
                pb_client=self._pb_client,
            )

    def __del__(self) -> None:
        self._tf.close()
