from __future__ import annotations

import dataclasses
import functools
import time
from typing import Optional

import numpy as np
from dm_robotics.geometry.geometry import Pose

from coffee import hints, ik
from coffee.client import BulletClient
from coffee.joints import Joints, JointState


@dataclasses.dataclass(frozen=True)
class ManipulatorConfig:
    """PyBullet manipulator configuration."""

    urdf: str
    """URDF file path."""

    max_joint_position_error: float
    """Threshold for joint position error."""

    max_joint_velocity_error: float
    """Threshold for joint velocity error."""

    manipulator_pose: Optional[Pose] = None
    """Pose of the manipulator in the world frame. If None, the manipulator is placed
    at the origin."""

    joint_resting_configuration: Optional[hints.Array] = None
    """The configuration of the joints at rest. These are useful for biasing the inverse
    kinematics solver solutions. If None, the mid-range of the joints is used."""

    ik_point_link_name: Optional[str] = None
    """The name of the link that will serve as the inverse kinematics target. If None,
    the link attached to the last controllable joint will be used."""

    fixed_base: bool = True
    """If True, forces the base of the manipulator to be static."""

    movement_timeout: float = 5.0
    """Maximum time to wait for a joint to reach a desired state (position, velocity,
    torque, etc.). This value should be tuned if movement speed is changed."""


@dataclasses.dataclass(frozen=False)
class Manipulator:
    """A robotic manipulator in PyBullet."""

    manipulator_config: ManipulatorConfig
    pb_client: BulletClient
    body_id: int
    joints: Joints
    ik_solver: ik.IKSolverProtocol

    @classmethod
    def create(
        cls,
        manipulator_config: ManipulatorConfig,
        pb_client: BulletClient,
    ) -> Manipulator:
        # NOTE(kevin): We might want to look at using additional flags here.
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        # Load the manipulator body from the URDF.
        body_id = pb_client.load_urdf(
            manipulator_config.urdf,
            pose=manipulator_config.manipulator_pose,
            useFixedBase=manipulator_config.fixed_base,
            flags=flags,
        )

        # Read the joint information.
        joints = Joints.from_body_id(
            body_id,
            pb_client,
            manipulator_config.joint_resting_configuration,
        )

        if manipulator_config.ik_point_link_name is None:
            ik_point_joint_id = joints.controllable_joints[-1]
        else:
            ik_point_joint_id = joints.get_joint_index_from_link_name(
                manipulator_config.ik_point_link_name
            )
        ik_solver = ik.SimpleIKSolver(
            pb_client=pb_client,
            joints=joints,
            ik_point_joint_id=ik_point_joint_id,
        )

        return cls(
            manipulator_config=manipulator_config,
            pb_client=pb_client,
            body_id=body_id,
            joints=joints,
            ik_solver=ik_solver,
        )

    def set_link_damping(self, linear: float, angular: float) -> None:
        """Set linear and angular damping of the manipulator links.

        Args:
            linear (float): The linear damping value, between 0 and 1.
            angular (float): The angular damping value, between 0 and 1.
        """
        assert 0.0 <= linear <= 1.0
        assert 0.0 <= angular <= 1.0

        for j in range(self.pb_client.getNumJoints(self.body_id)):
            self.pb_client.changeDynamics(
                bodyUniqueId=self.body_id,
                linkIndex=j,
                linearDamping=linear,
                angularDamping=angular,
            )

    def go_home(self, *args, **kwargs) -> bool:
        """Move the manipulator to its resting joint configuration.

        Returns:
            bool: Whether the command was executed successfully.
        """
        if self.manipulator_config.joint_resting_configuration is not None:
            q_home = self.manipulator_config.joint_resting_configuration
        else:
            q_home = self.joints.zeros_array()

        return self.set_joint_positions(
            q_home,
            *args,
            **kwargs,
        )

    def set_joint_positions(
        self,
        positions: hints.Array,
        blocking: bool = True,
        disable_dynamics: bool = False,
        kp: float = 3e-2,
        kv: float = 1.0,
    ) -> bool:
        positions = np.array(positions, copy=True)
        assert len(positions) == self.joints.dof

        # Clip to joint limits.
        positions = np.clip(
            a=positions,
            a_min=self.joints.joints_lower_limit,
            a_max=self.joints.joints_upper_limit,
        )

        if disable_dynamics:
            for i, joint_id in enumerate(self.joints.controllable_joints):
                self.pb_client.resetJointState(
                    self.body_id,
                    joint_id,
                    targetValue=positions[i],
                    targetVelocity=0.0,
                )
            return True

        success = False

        move_func = functools.partial(
            self.pb_client.setJointMotorControlArray,
            bodyIndex=self.body_id,
            jointIndices=self.joints.controllable_joints,
            # POSITION_CONTROL is an alias for CONTROL_MODE_POSITION_VELOCITY_PD.
            controlMode=self.pb_client.POSITION_CONTROL,
            # In POSITION_CONTROL mode, `forces` corresponds to the maximum motor force
            # used to reach the target value.
            forces=self.joints.joints_max_force,
            # In POSITION_CONTROL mode, the targetVelocity is not the maximum joint
            # velocity, but the desired velocity of the joint.
            targetVelocities=self.joints.zeros_array(),
            positionGains=self.joints.const_array(kp),
            velocityGains=self.joints.const_array(kv),
        )

        move_func(targetPositions=positions)

        if blocking:
            goal_positions = np.array(positions, copy=True)
            epsilon = self.manipulator_config.max_joint_position_error

            start_time = time.time()
            while True:
                if time.time() - start_time > self.manipulator_config.movement_timeout:
                    print("Unable to move joints in allotted time. Skipping...")
                    return False

                current_positions = self.get_joint_positions()
                diff = goal_positions - current_positions
                if np.all(np.abs(diff) < epsilon):
                    success = True

                # if not self.pb_client.realtime_mode:
                # self.pb_client.step()

                if success:
                    break

        return success

    def set_joint_velocities(
        self,
        velocities: hints.Array,
        blocking: bool = True,
    ) -> bool:
        velocities = np.array(velocities, copy=True)
        assert len(velocities) == self.joints.dof

        # Clip to joint limits.
        velocities = np.clip(
            a=velocities,
            a_min=-self.joints.joints_max_velocity,
            a_max=self.joints.joints_max_velocity,
        )

        success = False

        move_func = functools.partial(
            self.pb_client.setJointMotorControlArray,
            bodyIndex=self.body_id,
            jointIndices=self.joints.controllable_joints,
            controlMode=self.pb_client.VELOCITY_CONTROL,
            # NOTE: In velocity control mode, `forces` corresponds to the maximum motor
            # force used to reach the target value.
            forces=self.joints.joints_max_force,
            # forces=self.manipulator_config.max_joint_torques,
        )

        move_func(targetVelocities=velocities)

        if self.pb_client.realtime_mode and blocking:
            goal_velocities = np.array(velocities, copy=True)
            epsilon = self.manipulator_config.max_joint_velocity_error

            start_time = time.time()
            while True:
                if time.time() - start_time > self.manipulator_config.movement_timeout:
                    print("Unable to move joints in alloted time. Skipping...")
                    return False

                current_velocities = self.get_joint_velocities()
                diff = goal_velocities - current_velocities
                if np.all(np.abs(diff) < epsilon):
                    success = True

                if success:
                    break

        return success

    def set_eef_pose(
        self,
        pose: Pose,
        blocking: bool = True,
        disable_dynamics: bool = False,
        **kwargs,
    ) -> bool:
        joint_positions = self.ik_solver.solve(pose)

        if joint_positions is None:
            return False

        success = self.set_joint_positions(
            positions=joint_positions,
            blocking=blocking,
            disable_dynamics=disable_dynamics,
            **kwargs,
        )

        return success

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

    def get_eef_pose(self) -> Pose:
        return self.ik_solver.forward_kinematics()

    # Debugging.

    def debug_draw_frame(self, link_name: str, size: float = 0.1) -> None:
        froms = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        tos = [[size, 0, 0], [0, size, 0], [0, 0, size]]
        colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

        for lf, lt, lc in zip(froms, tos, colors):
            self.pb_client.addUserDebugLine(
                lineFromXYZ=lf,
                lineToXYZ=lt,
                lineColorRGB=lc,
                parentObjectUniqueId=self.body_id,
                parentLinkIndex=self.joints.get_joint_index_from_link_name(link_name),
            )

    def debug_draw_ik_point_frame(self, size: float = 0.1) -> None:
        self.debug_draw_frame(
            link_name=self.joints.get_link_name_from_joint_index(
                self.ik_solver.ik_point_joint_id
            ),
            size=size,
        )
