# import abc

# import numpy as np

# from coffee import body
# from coffee.client import BulletClient
# from coffee.joints import Joints


# class Arm(abc.ABC, body.Body):
#     """Base class for robotic arms."""

#     @abc.abstractmethod
#     def _build(self, *args, **kwargs) -> int:
#         """Arm initialization method to be overrided by subclasses."""

#     @property
#     @abc.abstractmethod
#     def joints(self) -> Joints:
#         """The arm's joints."""

#     @property
#     @abc.abstractmethod
#     def name(self) -> str:
#         """The name of the arm."""

#     @abc.abstractmethod
#     def set_joint_angles(
#         self,
#         pb_client: BulletClient,
#         joint_angles: np.ndarray,
#     ) -> None:
#         """Sets the joints of the robot to a given configuration."""
