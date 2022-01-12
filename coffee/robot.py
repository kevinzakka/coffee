import dataclasses
import tempfile
from typing import Optional

import numpy as np
from dm_robotics.geometry import geometry

from coffee import client, effector
from coffee.ik.ik_solver import IKSolver
from coffee.joints import Joints
from coffee.models.end_effectors.robot_hands import robot_hand
from coffee.models.robot_arms import robot_arm
from coffee.third_party.urdf_editor import urdf_editor


@dataclasses.dataclass
class Robot:
    """A Robot represents the union of an arm and a gripper."""

    pb_client: client.BulletClient
    arm: robot_arm.RobotArm
    gripper: robot_hand.RobotHand
    arm_effector: effector.Effector
    gripper_effector: Optional[effector.Effector]
    name: str = "robot"

    def __post_init__(self) -> None:
        self._arm_dof = self.arm.joints.dof
        self._gripper_dof = self.gripper.joints.dof

        # Create a combined joint resting configuration for the arm and gripper.
        # We assume that the gripper's configuration is all zeros.
        self._joint_resting_configuration = np.zeros(self._arm_dof + self._gripper_dof)
        self._joint_resting_configuration[
            : self._arm_dof
        ] = self.arm.joint_resting_configuration

        # Now attach the gripper to the arm.
        self._attach_gripper()

        # Setup the IK solver.
        self._gripper_ik_point_joint_id = self._joints.get_joint_index_from_link_name(
            self.gripper.tool_center_point
        )
        self._ik_solver = IKSolver(
            pb_client=self.pb_client,
            joints=self._joints,
            ik_point_joint_id=self._gripper_ik_point_joint_id,
            joint_damping=1e-5,
            nullspace_reference=self._joint_resting_configuration,
        )

    def __del__(self) -> None:
        # Close and delete the temporary URDF file.
        try:
            self._temp_urdf_file.close()
        except AttributeError:
            pass

    def _attach_gripper(self) -> None:
        """Attaches the gripper to the robot arm.

        PyBullet's `createConstraint` mechanism is wonky and broken. This function
        creates a new temporary URDF file by combining the arm's URDF with the gripper's
        URDF via a fixed joint. It then deletes the previous two bodies from the
        simulation and loads the new URDF file as the combined arm + gripper body.
        """
        arm_editor = urdf_editor.UrdfEditor()
        arm_editor.initializeFromBulletBody(self.arm.body_id, self.pb_client.client_id)

        hand_editor = urdf_editor.UrdfEditor()
        hand_editor.initializeFromBulletBody(
            self.gripper.body_id, self.pb_client.client_id
        )

        newjoint = arm_editor.joinUrdf(
            childEditor=hand_editor,
            parentLinkIndex=arm_editor.linkNameToIndex[self.arm.ik_point_link_name],
            jointPivotXYZInParent=[0, 0, 0],
            jointPivotRPYInParent=[np.pi, 0, 0],
            jointPivotXYZInChild=[0, 0, 0],
            jointPivotRPYInChild=[0, 0, 0],
            parentPhysicsClientId=self.pb_client.client_id,
            childPhysicsClientId=self.pb_client.client_id,
        )
        newjoint.joint_type = self.pb_client.JOINT_FIXED
        newjoint.joint_name = "joint_arm_gripper"

        # Combine arm + gripper URDFs into a temporary URDF. The file is deleted upon
        # object destruction, see `__del__`.
        self._temp_urdf_file = tempfile.NamedTemporaryFile(suffix=".urdf", delete=True)
        arm_editor.saveUrdf(self._temp_urdf_file.name)
        self._temp_urdf_file.flush()

        # Remove the arm and gripper bodies from the simulation.
        self.pb_client.remove_body(self.arm.body_id)
        self.pb_client.remove_body(self.gripper.body_id)

        # Load the combined arm + gripper body from the temporary URDF file.
        self._body_id = self.pb_client.load_urdf(
            filename=self._temp_urdf_file.name,
            fixed_base=self.arm._fixed_base,
        )

        # Store the joints of the combined arm + gripper body.
        self._joints = Joints.from_body_id(self._body_id, self.pb_client)

    @property
    def body_id(self) -> int:
        return self._body_id

    @property
    def joints(self) -> Joints:
        return self._joints

    @property
    def arm_dof(self) -> int:
        return self._arm_dof

    @property
    def gripper_dof(self) -> int:
        return self._gripper_dof

    # Methods.

    def position_arm_joints(self, joint_angles: np.ndarray) -> None:
        """Resets the arm joints to the desired configuration."""
        joint_angles = np.array(joint_angles)
        assert len(joint_angles) == self._arm_dof

        for i, joint_id in enumerate(self._joints.controllable_joints):
            if i < self._arm_dof:
                self.pb_client.resetJointState(
                    self._body_id,
                    joint_id,
                    targetValue=joint_angles[i],
                    targetVelocity=0.0,
                )

    def position_gripper(self, position: np.ndarray, quaternion: np.ndarray) -> None:
        """Resets the gripper IK point position to the desired pose."""
        # Use IK to map the desired Cartesian 6D pose to joint positions.
        joint_target = self._ik_solver.solve(geometry.Pose(position, quaternion))

        # If IK fails to find a solution, throw an error.
        if joint_target is None:
            raise ValueError(
                "IK Failed to converge to the desired target pose"
                f"{geometry.Pose(position, quaternion)}"
            )

        self.position_arm_joints(joint_target[: self._arm_dof])
