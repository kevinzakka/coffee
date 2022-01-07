from typing import Optional

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.models.robot_arms import robot_arm
from coffee.models.robot_arms import ur5_constants as consts


class UR5(robot_arm.RobotArm):
    """A UR5 arm from Universal Robots."""

    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "ur5",
        joint_resting_configuration: Array = consts.JOINT_RESTING_CONFIGURATION,
        ik_point_link_name: Optional[str] = consts.IK_POINT_LINK_NAME,
        fixed_base: bool = consts.FIXED_BASE,
        max_joint_position_error: float = consts.MAX_JOINT_POSITION_ERROR,
    ) -> None:
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        body_id = pb_client.load_urdf(
            str(consts.UR5_URDF),
            useFixedBase=fixed_base,
            flags=flags,
        )

        super().__init__(
            name=name,
            body_id=body_id,
            pb_client=pb_client,
            joint_resting_configuration=joint_resting_configuration,
            ik_point_link_name=ik_point_link_name,
            fixed_base=fixed_base,
            max_joint_position_error=max_joint_position_error,
        )
