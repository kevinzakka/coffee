# from coffee.models.arms import arm
# from coffee.models.arms import ur5_constants as consts


# class UR5(arm.Arm):
#     """A UR5 arm from Universal Robots."""

#     def _build(self) -> int:
#         flags = self.pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

#         body_id = self.pb_client.load_urdf(
#             consts.UR5_URDF,
#             useFixedBase=True,
#             flags=flags,
#         )

#         return body_id
