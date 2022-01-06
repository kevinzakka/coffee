from __future__ import annotations

import abc
from typing import Optional, Tuple

import numpy as np
from dm_robotics.geometry import geometry
from dm_robotics.transformations import transformations as tr

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.joints import Joints
from coffee.utils import geometry_utils


class Body(metaclass=abc.ABCMeta):
    """Abstract base class for a body.

    Subclasses must implement the `_build` method.
    """

    def __init__(self, pb_client: BulletClient, *args, **kwargs) -> None:
        """Body constructor.

        Subclasses should not override this method. Instead, they should implement a
        `_build` method.

        Args:
            pb_client: The BulletClient instance.
            *args: Positional arguments to pass to the `_build` method.
            **kwargs: Keyword arguments to pass to the `_build` method.
        """
        self._pb_client = pb_client

        self._body_id = self._build(*args, **kwargs)
        if self._body_id < 0:
            raise RuntimeError("Failed to create entity.")

        self._joints = Joints.from_body_id(self._body_id, self._pb_client)

    @abc.abstractmethod
    def _build(self, *args, **kwargs) -> int:
        """Body initialization method to be overridden by subclasses.

        Must return the unique body id returned by the bullet client.
        """

    # Accessors.

    @property
    def body_id(self) -> int:
        return self._body_id

    @property
    def pb_client(self) -> BulletClient:
        return self._pb_client

    @property
    def joints(self) -> Joints:
        return self._joints

    # Methods.

    def get_pose(self) -> geometry.Pose:
        """Gets the pose of the root link of the body in world coordinates."""
        position, quaternion = self.pb_client.getBasePositionAndOrientation(
            self.body_id
        )
        return geometry.Pose(
            position=position,
            quaternion=geometry_utils.as_quaternion_wxyz(quaternion),
        )

    def set_pose(
        self, position: Optional[Array] = None, quaternion: Optional[Array] = None
    ) -> None:
        """Sets the pose of the root link of the body in world coordinates.

        Note: This method overrides the effect of all simulation and the linear and
        angular velocities of the body are set to zero.

        Args:
            position (optional): The desired position of the body in world coordinates.
            quaternion (optional): The desired orientation of the body in world
                coordinates.
        """
        current_pose = self.get_pose()

        if position is None:
            position = current_pose.position
        if quaternion is None:
            quaternion = current_pose.quaternion

        self.pb_client.resetBasePositionAndOrientation(
            bodyUniqueId=self.body_id,
            posObj=position,
            ornObj=geometry_utils.as_quaternion_xyzw(quaternion),
        )

    def shift_pose(
        self,
        position: Optional[Array] = None,
        quaternion: Optional[Array] = None,
        rotate_velocity: bool = False,
    ) -> None:
        """Shifts the pose of the root link of the body from its current pose.

        The specified position is added to the current position and the specified
        quaternion is premultiplied to the current quaternion.

        Args:
            position (optional): The desired delta in position.
            quaternion (optional): Quaternion expressing the desired delta in rotation.
            rotate_velocity: Whether to shift the current linear velocity. This will
                rotate the current linear velocity, which is expressed relative to the
                world frame.
        """
        current_pose = self.get_pose()
        current_velocity = self.get_velocity()[0]
        new_position, new_quaternion = None, None

        if position is not None:
            assert len(position) == 3
            new_position = current_pose.position + position
        if quaternion is not None:
            assert len(quaternion) == 4
            quaternion = np.array(quaternion, dtype=np.float64, copy=False)
            new_quaternion = tr.quat_mul(quaternion, current_pose.quaternion)
        self.set_pose(new_position, new_quaternion)
        if rotate_velocity:
            new_velocity = tr.quat_rotate(quaternion, current_velocity)
            self.set_velocity(linear=new_velocity)

    def get_velocity(self) -> Tuple[Array, Array]:
        """Gets the linear and angular velocity of this body."""
        linear_velocity, angular_velocity = self.pb_client.getBaseVelocity(self.body_id)
        return np.asarray(linear_velocity), np.asarray(angular_velocity)

    def set_velocity(
        self, linear: Optional[Array] = None, angular: Optional[Array] = None
    ) -> None:
        """Sets the linear and angular velocity of this body.

        Args:
            linear (optional): The desired linear velocity of the body.
            angular (optional): The desired angular velocity of the body.
        """
        if linear is not None:
            assert len(np.asarray(linear)) == 3
            linear = np.asarray(linear).tolist()
        if angular is not None:
            assert len(np.asarray(angular)) == 3
            angular = np.asarray(angular).tolist()

        self.pb_client.resetBaseVelocity(
            objectUniqueId=self.body_id,
            linearVelocity=linear,
            angularVelocity=angular,
        )

    def configure_joints(self, position: Array) -> None:
        """Configures the joint positions of this body.

        Args:
            position: The desired position of the body's joints.
        """
        position = np.asarray(position)
        assert len(position) == self.joints.dof

        for i, joint_id in enumerate(self.joints.controllable_joints):
            self.pb_client.resetJointState(
                self.body_id,
                joint_id,
                targetValue=position[i],
                targetVelocity=0.0,
            )


class NamedBody(Body):
    """Base class for named bodies.

    A named body is a body with a unique name.
    """

    def __init__(self, name: str, pb_client: BulletClient, *args, **kwargs) -> None:
        self._name = name

        super().__init__(pb_client, *args, **kwargs)

    @property
    def name(self) -> str:
        return self._name
