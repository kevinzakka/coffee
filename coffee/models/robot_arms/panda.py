from typing import Optional

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.models.robot_arms import panda_constants as consts
from coffee.models.robot_arms import robot_arm


class Panda(robot_arm.RobotArm):
    """A Panda arm from Franka Emika."""

    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "panda",
        joint_resting_configuration: Array = consts.JOINT_RESTING_CONFIGURATION,
        ik_point_link_name: Optional[str] = consts.IK_POINT_LINK_NAME,
        fixed_base: bool = consts.FIXED_BASE,
        max_joint_position_error: float = consts.MAX_JOINT_POSITION_ERROR,
        enable_self_collision: bool = consts.ENABLE_SELF_COLLISION,
    ) -> None:
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        if enable_self_collision:
            flags |= pb_client.URDF_USE_SELF_COLLISION  # type: ignore

        body_id = pb_client.load_urdf(
            str(consts.PANDA_URDF),
            fixed_base=fixed_base,
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
