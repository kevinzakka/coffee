# import math
# import time

# from coffee import client
# from coffee.joints import Joints

# ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


# class Robotiq:
#     def __init__(self, urdf: str, pb_client: client.BulletClient) -> None:
#         self.pb_client = pb_client

#         self.id = self.pb_client.load_urdf(
#             urdf,
#             flags=pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
#             base_position=[0, 0, 0],
#         )

#         self.end_effector_joints = (
#             "finger_joint",
#             "left_inner_knuckle_joint",
#             "left_inner_finger_joint",
#             "right_outer_knuckle_joint",
#             "right_inner_knuckle_joint",
#             "right_inner_finger_joint",
#         )

#         self.gripper_range = [0, 0.085]
#         self.arm_num_dofs = 0
#         self.__parse_joint_info__()
#         self.__post_load__()

#     def __parse_joint_info__(self):
#         self.joints = Joints.from_body_id(self.id, self.pb_client)
#         self.controllable_joints = [
#             j
#             for j in self.joints.joint_info
#             if j.joint_type != self.pb_client.JOINT_FIXED
#         ]
#         self.arm_controllable_joints = [
#             j
#             for j in self.controllable_joints
#             if j.joint_name not in self.end_effector_joints
#         ]

#         for joint_info in self.joints.joint_info:
#             if joint_info.joint_type != self.pb_client.JOINT_FIXED:
#                 self.pb_client.setJointMotorControl2(
#                     self.id,
#                     joint_info.joint_index,
#                     self.pb_client.VELOCITY_CONTROL,
#                     targetVelocity=0,
#                     force=0,
#                 )

#     def __post_load__(self):
#         mimic_parent_name = "finger_joint"
#         mimic_children_names = {
#             "right_outer_knuckle_joint": 1,
#             "left_inner_knuckle_joint": 1,
#             "right_inner_knuckle_joint": 1,
#             "left_inner_finger_joint": -1,
#             "right_inner_finger_joint": -1,
#         }
#         self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)

#     def __setup_mimic_joints__(self, mimic_parent_name, mimic_children_names):
#         self.mimic_parent_id = [
#             joint.joint_index
#             for joint in self.joints.joint_info
#             if joint.joint_name == mimic_parent_name
#         ][0]
#         self.mimic_child_multiplier = {
#             joint.joint_index: mimic_children_names[joint.joint_name]
#             for joint in self.joints.joint_info
#             if joint.joint_name in mimic_children_names
#         }
#         for joint_id, multiplier in self.mimic_child_multiplier.items():
#             c = self.pb_client.createConstraint(
#                 self.id,
#                 self.mimic_parent_id,
#                 self.id,
#                 joint_id,
#                 jointType=self.pb_client.JOINT_GEAR,
#                 jointAxis=[0, 1, 0],
#                 parentFramePosition=[0, 0, 0],
#                 childFramePosition=[0, 0, 0],
#             )
#             self.pb_client.changeConstraint(
#                 c,
#                 gearRatio=-multiplier,
#                 maxForce=100,
#                 erp=1,
#             )  # Note: the mysterious `erp` is of EXTREME importance

#     def move_gripper(self, open_length):
#         open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)
#         self.pb_client.setJointMotorControl2(
#             self.id,
#             self.mimic_parent_id,
#             self.pb_client.POSITION_CONTROL,
#             targetPosition=open_angle,
#             force=self.joints.joint_info[self.mimic_parent_id].joint_max_force,
#             maxVelocity=self.joints.joint_info[self.mimic_parent_id].joint_max_velocity,
#         )

#     def close_gripper(self):
#         self.move_gripper(self.gripper_range[0])

#     def open_gripper(self):
#         self.move_gripper(self.gripper_range[1])


# def main() -> None:
#     # Create the bullet client.
#     bullet_client = client.BulletClient.create(
#         client_config=client.ClientConfig(gui=True, realtime=True),
#         physics_config=client.PhysicsConfig(),
#     )
#     bullet_client.setAdditionalSearchPath(ASSETS_PATH)

#     # Load plane.
#     bullet_client.load_urdf("plane/plane.urdf")

#     with bullet_client.disable_rendering():
#         gripper = Robotiq(
#             urdf="robots/robotiq/robotiq_2f_85.urdf",
#             pb_client=bullet_client,
#         )

#     gripper.open_gripper()
#     for _ in range(100):
#         bullet_client.step_simulation()
#         time.sleep(0.01)
#     gripper.close_gripper()
#     for _ in range(100):
#         bullet_client.step_simulation()
#         time.sleep(0.01)


# if __name__ == "__main__":
#     main()
