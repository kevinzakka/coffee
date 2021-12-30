import numpy as np

from coffee import client
from coffee.ik.ik_solver import IKSolver, IKConfig
from coffee.joints import Joints

ASSETS_PATH = "/Users/kevin/repos/coffee/vendor"


def main():
    # Create the bullet client.
    bullet_client = client.BulletClient.create(
        client_config=client.ClientConfig(realtime=True),
    )
    bullet_client.setAdditionalSearchPath(ASSETS_PATH)

    body_id = bullet_client.load_urdf("robots/universal_robot/ur_description/ur5.urdf")

    joint_resting_configuration = [
        j * np.pi for j in [-1.0, -0.5, 0.5, -0.5, -0.5, 0.0]
    ]
    joints = Joints.from_body_id(body_id, bullet_client, joint_resting_configuration)

    ik_solver = IKSolver(
        pb_client=bullet_client,
        body_id=body_id,
        ik_point_joint_id=joints.get_joint_index_from_link_name("ee_link"),
        joints=joints,
        ik_config=IKConfig(),
    )

    for i in range(5):
        # Generate a random joint configuration and reset joints to it.
        joint_config = np.random.randn(joints.dof)

        for i, joint_id in enumerate(joints.controllable_joints):
            bullet_client.resetJointState(
                body_id,
                joint_id,
                joint_config[i],
            )

        # Solve for eef pose.
        pose = ik_solver.forward_kinematics()

        # Solve for joint configuration given eef pose.
        target_joints = ik_solver.solve(pose, use_nullspace=True)

        # phases = (phases + np.pi) % (2 * np.pi) - np.pi

        assert target_joints.max() <= (2 * np.pi)
        assert target_joints.min() >= (-2 * np.pi)

    bullet_client.disconnect()


if __name__ == "__main__":
    main()
