# import os
# import time
# import numpy as np
# from dm_robotics.transformations import transformations as tr
# from coffee import client
# from coffee.models.end_effectors.robot_hands.schunk_wsg_50 import SchunkWsg50
# from coffee.models.end_effectors.robot_hands.rg2 import RG2
# from coffee.models.robot_arms.ur5 import UR5

# from coffee.urdf_editor import UrdfEditor


# def debug_draw_frame(pb_client, body_id, link_index, size: float = 0.1) -> None:
#     froms = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
#     tos = [[size, 0, 0], [0, size, 0], [0, 0, size]]
#     colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
#     for lf, lt, lc in zip(froms, tos, colors):
#         pb_client.addUserDebugLine(
#             lineFromXYZ=lf,
#             lineToXYZ=lt,
#             lineColorRGB=lc,
#             parentObjectUniqueId=body_id,
#             parentLinkIndex=link_index,
#         )


# def main(bullet_client: client.BulletClient) -> None:
#     with bullet_client.disable_rendering():
#         arm = UR5(bullet_client)
#         arm.configure_joints(arm._joint_resting_configuration)

#         ed_arm = UrdfEditor()
#         ed_arm.initializeFromBulletBody(arm.body_id, bullet_client.client_id)

#         # hand = SchunkWsg50(bullet_client, scaling=0.6)
#         # hand.reset_pose([0.5, -0.5, 0.1])

#         hand = RG2(bullet_client, scaling=1.0)
#         # hand.reset_pose([0.0, 0.0, 0.3], quaternion=[0, 1, 1, 0])

#         ed_hand = UrdfEditor()
#         ed_hand.initializeFromBulletBody(hand.body_id, bullet_client.client_id)

#         newjoint = ed_arm.joinUrdf(
#             childEditor=ed_hand,
#             parentLinkIndex=7,
#             jointPivotXYZInParent=[0, 0.055, 0],
#             jointPivotRPYInParent=[-np.pi / 2, 0, 0],
#             jointPivotXYZInChild=[0, 0, 0],
#             jointPivotRPYInChild=[0, 0, 0],
#             parentPhysicsClientId=bullet_client.client_id,
#             childPhysicsClientId=bullet_client.client_id,
#         )
#         newjoint.joint_type = bullet_client.JOINT_FIXED
#         newjoint.joint_name = "joint_arm_gripper"
#         urdfname = f".poop.urdf"
#         ed_arm.saveUrdf(urdfname)
#         bullet_client.removeBody(arm.body_id)
#         bullet_client.removeBody(hand.body_id)

#         body_id = bullet_client.loadURDF(
#             urdfname,
#             useFixedBase=True,
#             basePosition=[0, 0, 0],
#             baseOrientation=[0, 0, 0, 1],
#         )
#         os.remove(urdfname)

#     debug_draw_frame(bullet_client, arm.body_id, 6, size=0.1)
#     debug_draw_frame(bullet_client, hand.body_id, 1, size=0.1)

#     # constraint = bullet_client.createConstraint(
#     #     parentBodyUniqueId=arm.body_id,
#     #     parentLinkIndex=6,
#     #     childBodyUniqueId=hand.body_id,
#     #     childLinkIndex=0,
#     #     jointType=bullet_client.JOINT_FIXED,
#     #     jointAxis=[0, 0, 0],
#     #     parentFramePosition=[0.0, 0.0, 0.0],
#     #     childFramePosition=[0.0, 0.0, 0.0],
#     #     childFrameOrientation=[0, 1, 0, 0],
#     # )

#     # p2 = props.Box(bullet_client, color=(1, 0.3412, 0.3490, 1), half_extents=[0.02, 0.02, 0.02])
#     # p2.reset_pose([0.5, 0.0, 0.1])

#     while True:
#     # arm.configure_joints(arm._joint_resting_configuration)
#         hand._set_constraints()
#         bullet_client.step()
#         time.sleep(0.01)

#     bullet_client.infinite_step()


# if __name__ == "__main__":
#     bullet_client = client.BulletClient.create(
#         mode=client.ConnectionMode.GUI,
#         config=client.ClientConfig(realtime=True),
#     )
#     bullet_client.load_urdf("plane/plane.urdf")
#     main(bullet_client)
#     bullet_client.disconnect()
