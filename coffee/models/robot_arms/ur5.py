# import numpy as np

# from coffee.client import BulletClient
# from coffee.models.robot_arms import robot_arm
# from coffee.models.robot_arms import ur5_constants as consts


# class UR5(robot_arm.RobotArm):
#     """A UR5 arm from Universal Robots."""

#     def _build(
#         self,
#         fix_base: bool = True,
#         *args,
#         **kwargs,
#     ) -> int:
#         del args, kwargs

#         flags = self.pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

#         body_id = self.pb_client.load_urdf(
#             str(consts.UR5_URDF),
#             useFixedBase=fix_base,
#             flags=flags,
#         )

#         return body_id

#     def set_joint_angles(
#         self,
#         pb_client: BulletClient,
#         joint_angles: np.ndarray,
#     ) -> None:
#         """Sets the joints of the robot to a given configuration."""
