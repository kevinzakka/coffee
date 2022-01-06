from __future__ import annotations

import dataclasses
from typing import NamedTuple, Optional

import numpy as np
import pybullet as p
from dm_robotics.geometry.geometry import Pose
from dm_robotics.transformations import transformations as tr

from coffee.client import BulletClient, ClientConfig, ConnectionMode
from coffee.joints import Joints, LinkState
from coffee.utils import geometry_utils


class IKSolution(NamedTuple):
    """An IK solution returned by the IKSolver."""

    qpos: np.ndarray
    """The joint configuration."""
    linear_err: float
    """The linear error between the solved pose and the target pose."""
    angular_err: float
    """The angular error between the solved pose and the target pose."""


@dataclasses.dataclass
class IKSolver:
    """Inverse kinematics solver.

    Computes a joint configuration that brings an element (in a kinematic chain) to a
    desired pose. Uses PyBullet's `calculateInverseKinematics` method under the hood,
    which implements damped least squares (aka Levenberg-Marquardt) IK.
    """

    pb_client: BulletClient
    """The pybullet client."""

    joints: Joints
    """The joints used to achieve the desired target pose."""

    ik_point_joint_id: int
    """The id of the joint attached to the link being placed by the IK solver."""

    joint_damping: float = 0.0
    """The damping coefficient used to stabilize the IK solution."""

    nullspace_joint_position_reference: Optional[np.ndarray] = None
    """"""

    def __post_init__(self) -> None:
        if self.nullspace_joint_position_reference is None:
            self.nullspace_joint_position_reference = 0.5 * np.sum(
                self.joints.joints_range, axis=1
            )

        # Dirty hack to get around pybullet's lack of support for computing FK given a
        # joint configuration as an argument.
        # See: https://github.com/bulletphysics/bullet3/issues/2603
        self._shadow_client = BulletClient.create(
            mode=ConnectionMode.DIRECT,
            config=ClientConfig(realtime=False, render_shadows=False),
        )
        manipulator_kwargs = self.pb_client._body_cache[self.joints.body_id]
        shadow_body_id = self._shadow_client.load_urdf(**manipulator_kwargs)
        self._shadow_joints = Joints.from_body_id(shadow_body_id, self._shadow_client)

    def solve(
        self,
        ref_pose: Pose,
        linear_tol: float = 1e-4,
        angular_tol: float = 1e-4,
        max_steps: int = 100,
        num_attempts: int = 100,
        stop_on_first_successful_attempt: bool = False,
        inital_joint_configuration: Optional[np.ndarray] = None,
        nullspace_reference: Optional[np.ndarray] = None,
        verbose: bool = False,
    ) -> Optional[np.ndarray]:
        """Attempts to solve the inverse kinematics problem.

        This method computes a joint configuration that solves the IK problem. It
        returns None if no solution is found. If multiple solutions are
        found, it will return the one where the joints are closer to the
        `nullspace_reference`. If no `nullspace_reference is provided, it will use the
        center of the joint ranges as reference.

        Args:
            ref_pose: Target pose of the controlled element in Cartesian world frame.
            linear_tol: The linear tolerance, in meters, that determines if the solution
                found is valid.
            angular_tol: The angular tolerance, in radians, that determines if the
                solution found is valid.
            max_steps:
            num_attempts: The number of different attempts the solver should do. For a
                given target pose, there exists an infinite number of possible
                solutions, having more attempts allows to compare different joint
                configurations. The solver will return the solution where the joints are
                closer to the `nullspace_reference`. Note that not all attempts are
                successful, and thus, having more attempts gives better chances of
                finding a correct solution.
            stop_on_first_successful_attempt: If true, the method will return the
                first solution that meets the tolerance criteria. If false, returns the
                solution where the joints are closer to `nullspace_reference`.
            inital_joint_configuration: A joint configuration that will be used for
                the first attempt. This can be useful in the case of a complex pose,
                a user could provide the initial guess that is close to the desired
                solution. If None, all the joints will be set to 0 for the first
                attempt.
            nullspace_reference: The desired joint configuration. When the controlled
                element is in the desired pose, the solver will try and bring the joint
                configuration closer to the nullspace reference without moving the
                element. If no nullspace reference is provided, the center of the joint
                ranges is used as reference.

        Returns:
            The corresponding joint configuration if a solution is found, else None.

        Raises:
            ValueError: If the `nullspace_reference` does not have the correct length.
            ValueError: If the `inital_joint_configuration` does not have the correct
                length.
        """
        if nullspace_reference is None:
            nullspace_reference = self.nullspace_joint_position_reference
        else:
            if len(nullspace_reference) != self.joints.dof:
                raise ValueError("nullspace_reference has an invalid length.")

        if inital_joint_configuration is None:
            inital_joint_configuration = self.joints.zeros_array()
        else:
            inital_joint_configuration = np.array(inital_joint_configuration)
            if len(inital_joint_configuration) != self.joints.dof:
                raise ValueError("inital_joint_configuration has an invalid length.")

        nullspace_jnt_qpos_min_err = np.inf
        sol_qpos = None
        success = False

        # Each iteration of this loop attempts to solve the inverse kinematics.
        # If a solution is found, it is compared to previous solutions.
        for attempt in range(num_attempts):
            # Use the user provided joint configuration for the first attempt.
            if attempt == 0:
                qpos_new = inital_joint_configuration
            else:
                # Randomize the initial joint configuration so that the IK can find
                # a different solution.
                qpos_new = np.random.uniform(
                    low=self.joints.joints_lower_limit,
                    high=self.joints.joints_upper_limit,
                )

            # Reset the joints to this configuration.
            for i, joint_id in enumerate(self._shadow_joints.controllable_joints):
                self._shadow_client.resetJointState(
                    self._shadow_joints.body_id,
                    joint_id,
                    qpos_new[i],
                )

            # Solve the IK.
            joint_qpos, linear_err, angular_err = self._solve_ik(
                ref_pose,
                max_steps,
                verbose,
            )

            # Check if the attempt was successful. The solution is saved if the
            # joints are closer to the nullspace reference.
            if linear_err <= linear_tol and angular_err <= angular_tol:
                success = True
                nullspace_jnt_qpos_err = float(
                    np.linalg.norm(joint_qpos - nullspace_reference)
                )
                if nullspace_jnt_qpos_err < nullspace_jnt_qpos_min_err:
                    nullspace_jnt_qpos_min_err = nullspace_jnt_qpos_err
                    sol_qpos = joint_qpos

            if verbose:
                print(
                    f"attempt: {attempt} - nullspace_jnt_qpos_min_err: {nullspace_jnt_qpos_min_err} - success: {success}"
                )

            if success and stop_on_first_successful_attempt:
                break

        if not success:
            print(f"Unable to solve inverse kinematics for ref_pose: {ref_pose}")
        else:
            if verbose:
                print(f"Found a solution in {attempt} attempts.")

        return sol_qpos

    def _solve_ik(
        self,
        ref_pose: Pose,
        max_steps: int,
        verbose: bool,
    ) -> IKSolution:
        """Finds a joint configuration that brings element pose to target pose."""
        try:
            qpos = self._shadow_client.calculateInverseKinematics(
                bodyUniqueId=self._shadow_joints.body_id,
                endEffectorLinkIndex=self.ik_point_joint_id,
                targetPosition=ref_pose.position,
                targetOrientation=geometry_utils.as_quaternion_xyzw(
                    ref_pose.quaternion
                ),
                residualThreshold=1e-5,
                maxNumIterations=max_steps,
                jointDamping=self._shadow_joints.const_array(
                    self.joint_damping
                ).tolist(),
            )
            if np.isnan(np.sum(qpos)):
                qpos = None
            else:
                # Clip to joint limits.
                qpos = np.clip(
                    a=qpos,
                    a_min=self._shadow_joints.joints_lower_limit,
                    a_max=self._shadow_joints.joints_upper_limit,
                )
        except p.error as e:
            if verbose:
                print(f"IK failed with error message: {e}")
            qpos = None

        # If we were unable to find a solution, exit early.
        if qpos is None:
            return IKSolution(np.empty(self._shadow_joints.dof), np.inf, np.inf)

        # If we found a solution, we compute its associated pose and compare with the
        # target pose. We do this by first using forward kinematics to compute the
        # pose of the controlled element associated with the solution and then computing
        # linear and angular errors.

        # Forward kinematics.
        for i, joint_id in enumerate(self._shadow_joints.controllable_joints):
            self._shadow_client.resetJointState(
                self._shadow_joints.body_id,
                joint_id,
                qpos[i],
            )
        cur_pose = self.forward_kinematics(shadow=True)

        # Error computation.
        linear_err = float(np.linalg.norm(ref_pose.position - cur_pose.position))
        err_quat = tr.quat_diff_active(ref_pose.quaternion, cur_pose.quaternion)
        err_axis_angle = tr.quat_to_axisangle(err_quat)
        angular_err = float(np.linalg.norm(err_axis_angle))

        return IKSolution(np.array(qpos), linear_err, angular_err)

    def forward_kinematics(self, shadow: bool = False) -> Pose:
        if shadow:
            eef_link_state = LinkState(
                *self._shadow_client.getLinkState(
                    bodyUniqueId=self._shadow_joints.body_id,
                    linkIndex=self.ik_point_joint_id,
                    computeLinkVelocity=0,
                    computeForwardKinematics=True,
                )
            )
        else:
            eef_link_state = LinkState(
                *self.pb_client.getLinkState(
                    bodyUniqueId=self.joints.body_id,
                    linkIndex=self.ik_point_joint_id,
                    computeLinkVelocity=0,
                    computeForwardKinematics=True,
                )
            )

        return Pose(
            position=eef_link_state.link_world_position,
            quaternion=geometry_utils.as_quaternion_wxyz(
                eef_link_state.link_world_orientation
            ),
        )
