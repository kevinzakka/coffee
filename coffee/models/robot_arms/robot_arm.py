# import abc
# import functools

# import numpy as np

# from coffee import body
# from coffee.client import BulletClient
# from coffee.joints import Joints


# class RobotArm(abc.ABC, body.NamedBody):
#     """Base class for robot arms."""

#     def __init__(self, name: str, pb_client: BulletClient) -> None:
#         self._name = name
#         self._pb_client = pb_client

#         self._body_id = self._build(*args, **kwargs)
#         if self._body_id < 0:
#             raise RuntimeError("Failed to create entity.")

#         self._joints = Joints.from_body_id(self._body_id, self._pb_client)

#     @abc.abstractmethod
#     def _build(self, *args, **kwargs) -> int:
#         """Initialization method to be overrided by subclasses."""

#     @property
#     def joints(self) -> Joints:
#         """The joints of the arm."""
#         return self._joints

#     @abc.abstractmethod
#     def set_joint_angles(
#         self,
#         pb_client: BulletClient,
#         joint_angles: np.ndarray,
#     ) -> None:
#         """Sets the joints of the robot to a given configuration."""
#         joint_angles = np.array(joint_angles, copy=True)
#         assert len(joint_angles) == self.joints.dof

#         move_func = functools.partial(
#             self.pb_client.setJointMotorControlArray,
#             bodyIndex=self.body_id,
#             jointIndices=self.joints.controllable_joints,
#             # POSITION_CONTROL is an alias for CONTROL_MODE_POSITION_VELOCITY_PD.
#             controlMode=self.pb_client.POSITION_CONTROL,
#             # In POSITION_CONTROL mode, `forces` corresponds to the maximum motor force
#             # used to reach the target value.
#             forces=self.joints.joints_max_force,
#             # In POSITION_CONTROL mode, the targetVelocity is not the maximum joint
#             # velocity, but the desired velocity of the joint.
#             targetVelocities=self.joints.zeros_array(),
#             positionGains=self.joints.const_array(consts.DEFAULT_Kp),
#             velocityGains=self.joints.const_array(consts.DEFAULT_Kv),
#         )

#         move_func(targetPositions=positions)
