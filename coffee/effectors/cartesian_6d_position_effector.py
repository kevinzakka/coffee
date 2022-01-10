# Some implementation details regarding PyBullet:
#
# Each joint has a velocity motor. To disable the motor, enter velocity control mode and
# set the max force to 0. This allows the joint to be controlled via torques.
#
# In each simulation step, the physics engine will simulate the motors to reach the
# given target value that can be reached within the maximum motor forces and other
# constraints.
#
# A joint motor is controlled via the `setJointMotorControl2` method. To control
# multiple joint motors at once, use the `setJointMotorControlArray` method.
#
# Joint Position Control:
# =======================
# Input: desired joint positions `qpos_des`
# Requirements:
#   maximum time limit
#   maximum joint error
#   physical limitations: joint ranges, and maximum joint torques and velocities
# How? POSITION_CONTROL mode is an alias for CONTROL_MODE_POSITION_VELOCITY_PD, so
# pybullet actually uses a PD controller under the hood of `setJointMotorControl2`.

import dataclasses
import time
from typing import Optional

import numpy as np
from dm_robotics.geometry import geometry

from coffee import effector, structs
from coffee.client import BulletClient
from coffee.ik.ik_solver import IKSolver
from coffee.joints import Joints
from coffee.utils.geometry_utils import l2_normalize


@dataclasses.dataclass
class ModelParams:
    """Helper class to store model parameters for `Cartesian6dPositionEffector`.

    Attributes:
        joints: The joints of body to control.
        link_id: The id of the link to be controlled. The Cartesian 6 DoF poses fed to
            the effector are about this link's origin with respect to the world frame.
    """

    joints: Joints
    link_id: int


@dataclasses.dataclass
class ControlParams:
    """Helper class to store control parameters for `Cartesian6dPositionEffector`.

    Attributes:
        speed: The desired speed of the body.
        max_control_timesteps: The maximum amount of time in seconds the effector
            will spend controlling the joints of the body.
        max_joint_position_error: The maximum absolute error that is allowed between
            the joint positions of the body and the joint positions corresponding to
            the desired Cartesian 6 DoF pose.
        nullspace_reference: The joint positions to use as a nullspace reference for the
            IK solver. If None, the IK solver will use the mid-range of the joints'
            range of motion.
        joint_damping: The joint damping coefficient used by the IK solver.
    """

    max_control_timesteps: float
    speed: float = 1e-3
    max_joint_position_error: float = 1e-3
    nullspace_reference: Optional[np.ndarray] = None
    joint_damping: float = 1e-3


class Cartesian6dPositionEffector(effector.Effector):
    """A Cartesian 6D position effector interface for a robot arm.

    This effector uses PyBullet's underlying PD controller to position
    """

    def __init__(
        self,
        model_params: ModelParams,
        control_params: ControlParams,
        pb_client: BulletClient,
    ) -> None:
        """Initializes a PD control-based 6D Cartesian position effector.

        Args:
            model_params: A `ModelParams` instance.
            control_params: A `ControlParams` instance.
            pb_client: A `BulletClient` instance.
        """
        super().__init__(pb_client=pb_client)

        self._model_params = model_params
        self._control_params = control_params

        # Cache some variables for easier access.
        self._joints = model_params.joints
        self._body_id = self._joints.body_id
        self._joints_lower_limit = self._joints.joints_lower_limit
        self._joints_upper_limit = self._joints.joints_upper_limit
        self._joints_max_velocity = self._joints.joints_max_velocity
        self._joints_max_force = self._joints.joints_max_force
        self._joint_ids = self._joints.controllable_joints
        self._control_mode = self._pb_client.POSITION_CONTROL
        self._max_control_timesteps = control_params.max_control_timesteps
        self._max_joint_position_error = control_params.max_joint_position_error
        self._speed = control_params.speed

        # Setup the IK solver.
        self._ik_solver = IKSolver(
            pb_client=pb_client,
            joints=self._joints,
            ik_point_joint_id=self._model_params.link_id,
            joint_damping=control_params.joint_damping,
            nullspace_reference=control_params.nullspace_reference,
        )

    def set_control(self, command: np.ndarray) -> None:
        """Sets a 6 DoF Cartesian position command at the current timestep.

        Args:
            command: An array of 7 elements, representing the desired 6 DoF Cartesian
                pose: [pos, quat].
        """
        if command.size != 7:
            raise ValueError("set_control: command must be an np.ndarray of size 7.")

        cartesian_6d_target = np.copy(command)

        # Use IK to map the desired Cartesian 6D pose to joint positions.
        joint_target = self._ik_solver.solve(
            geometry.Pose(cartesian_6d_target[:3], cartesian_6d_target[3:])
        )

        # IK can fail to find a solution, in which case we exit.
        if joint_target is None:
            print("set_control: IK failed to find a solution.")
            return

        t0 = time.time()
        while (time.time() - t0) <= self._max_control_timesteps:
            # Break if within joint position error tolerance.
            current_positions = self._get_current_joint_positions()
            diff = joint_target - current_positions
            if np.all(np.abs(diff) < self._max_joint_position_error):
                break

            # Compute target joint positions such that the body moves with constant
            # velocity.
            direction = l2_normalize(diff)
            target_positions = current_positions + self._speed * direction

            self._pb_client.setJointMotorControlArray(
                bodyIndex=self._body_id,
                jointIndices=self._joint_ids,
                controlMode=self._control_mode,
                # In POSITION_CONTROL mode, forces corresponds to the maximum motor
                # force used to reach the target value.
                forces=self._joints_max_force,
                targetPositions=target_positions,
                # positionGains = the Kp term of the underlying PD controller.
                positionGains=self._joints.ones_array(),
            )
            self._pb_client.step()

    def _get_current_joint_positions(self) -> np.ndarray:
        """Returns the joint positions of the body at the current timestep."""
        states = [
            structs.JointState(*js)
            for js in self._pb_client.getJointStates(
                self._body_id, self._joints.controllable_joints
            )
        ]
        return np.array([state.joint_position for state in states])
