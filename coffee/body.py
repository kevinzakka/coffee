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

    def get_pose(self) -> geometry.Pose:
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
        linear_velocity, angular_velocity = self.pb_client.getBaseVelocity(self.body_id)
        return np.asarray(linear_velocity), np.asarray(angular_velocity)

    def set_velocity(
        self, linear: Optional[Array] = None, angular: Optional[Array] = None
    ) -> None:
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
        super().__init__(body_id=body_id, pb_client=pb_client)

        self._name = name

    @property
    def name(self) -> str:
        return self._name
