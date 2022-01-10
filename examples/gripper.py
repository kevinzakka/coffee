from dm_robotics.geometry import geometry

from coffee import client
from coffee.ik.ik_solver import IKSolver

# from coffee.ik.simple_ik_solver import SimpleIKSolver
from coffee.models import robot_arms
from coffee.models.end_effectors.robot_hands.schunk_wsg_50 import SchunkWsg50


def main(bullet_client: client.BulletClient) -> None:
    with bullet_client.disable_rendering():
        arm = robot_arms.UR5(bullet_client)
        # arm = robot_arms.xArm7(bullet_client)
        # arm = robot_arms.IIWA(bullet_client)
        arm.configure_joints(arm._joint_resting_configuration)

        hand = SchunkWsg50(bullet_client, scaling=0.6)
        hand.reset_pose([0.5, -0.5, 0.1])

        arm.attach(hand)

    ik_solver = IKSolver(
        bullet_client,
        arm.joints,
        arm.ik_point_link_id,
        joint_damping=1e-3,
        nullspace_reference=arm._joint_resting_configuration,
    )
    # ik_solver = SimpleIKSolver(
    #     pb_client=bullet_client,
    #     joints=arm.joints,
    #     ik_point_joint_id=arm.ik_point_link_id,
    #     joint_damping=1e-3,
    # )

    bullet_client.infinite_step()

    eef_pose = geometry.Pose(position=[0.5, 0, 0.2])
    qpos = ik_solver.solve(eef_pose)
    if qpos is not None:
        arm.configure_joints(qpos)
    bullet_client.infinite_step()


if __name__ == "__main__":
    bullet_client = client.BulletClient.create(
        mode=client.ConnectionMode.GUI,
        config=client.ClientConfig(),
    )
    bullet_client.load_urdf("objects/plane/plane.urdf")
    main(bullet_client)
    bullet_client.disconnect()
