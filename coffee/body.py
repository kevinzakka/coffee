from __future__ import annotations

import abc
from typing import Optional, Tuple

import numpy as np
from dm_robotics.transformations import transformations as tr

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.joints import Joints
from coffee.utils import geometry_utils


class Body(metaclass=abc.ABCMeta):
    """Abstract base class for a body."""

    @abc.abstractmethod
    def __init__(self, body_id: int, pb_client: BulletClient) -> None:
        """Body constructor.

        Args:
            body_id: The unique ID of the body.
            pb_client: The BulletClient instance.
        """
        if body_id < 0:
            raise ValueError("Body ID must be non-negative.")

        self._body_id = body_id
        self._pb_client = pb_client

        self._joints = Joints.from_body_id(body_id=body_id, pb_client=pb_client)

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

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        position, quaternion = self.pb_client.getBasePositionAndOrientation(
            self.body_id
        )
        position = np.asarray(position)
        quaternion = geometry_utils.as_quaternion_wxyz(quaternion)
        return position, quaternion

    def reset_pose(
        self, position: Optional[Array] = None, quaternion: Optional[Array] = None
    ) -> None:
        """Reset the position and orientation of the body root in the world frame.

        This also resets the linear and angular velocities of the body.

        If one of position/orientation is None, it is not reset.
        """
        current_position, current_quaternion = self.get_pose()

        if position is None:
            position = current_position
        if quaternion is None:
            quaternion = current_quaternion

        quaternion_xyzw = geometry_utils.as_quaternion_xyzw(quaternion)

        self.pb_client.resetBasePositionAndOrientation(
            bodyUniqueId=self.body_id,
            posObj=position,
            ornObj=quaternion_xyzw,
        )

    def shift_pose(
        self,
        position: Optional[Array] = None,
        quaternion: Optional[Array] = None,
    ) -> None:
        """Shifts the position and/or the orientation of the body from its current pose.

        This is a convenience function that wraps `set_pose`. The specified position is
        added to the current body position, and the specified quaternion is pre
        multiplied to the current body quaternion.
        """
        current_position, current_quaternion = self.get_pose()
        new_position, new_quaternion = None, None

        if position is not None:
            new_position = current_position + position
        if quaternion is not None:
            quaternion = np.array(quaternion, dtype=np.float64, copy=False)
            new_quaternion = tr.quat_mul(quaternion, current_quaternion)

        self.reset_pose(new_position, new_quaternion)

    def get_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        linear_velocity, angular_velocity = self.pb_client.getBaseVelocity(self.body_id)
        linear_velocity = np.asarray(linear_velocity)
        angular_velocity = np.asarray(angular_velocity)
        return linear_velocity, angular_velocity

    def reset_velocity(
        self, linear: Optional[Array] = None, angular: Optional[Array] = None
    ) -> None:
        """Reset the linear and angular velocities of the body.

        If one of the arguments is None, the corresponding velocity is reset to 0.
        """
        if linear is not None:
            if len(np.asarray(linear)) != 3:
                raise ValueError("Linear velocity must be a length 3 vector.")
            linear = np.asarray(linear)
        else:
            linear = np.zeros(3)
        if angular is not None:
            if len(np.asarray(angular)) != 3:
                raise ValueError("Angular velocity must be a length 3 vector.")
            angular = np.asarray(angular)
        else:
            angular = np.zeros(3)

        self.pb_client.resetBaseVelocity(self.body_id, linear, angular)

    def configure_joints(self, position: Array) -> None:
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
    """A `Body` with a unique name."""

    @abc.abstractmethod
    def __init__(self, body_id: int, name: str, pb_client: BulletClient) -> None:
        """NamedBody constructor.

        Args:
            body_id: The unique ID of the body.
            name: The unique name of the body.
            pb_client: The BulletClient instance.
        """
        super().__init__(body_id=body_id, pb_client=pb_client)

        self._name = name

    @property
    def name(self) -> str:
        return self._name
