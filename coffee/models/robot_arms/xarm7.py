from typing import Optional

import numpy as np

from coffee.client import BulletClient
from coffee.hints import Array
from coffee.models.robot_arms import robot_arm
from coffee.models.robot_arms import xarm7_constants as consts


class xArm7(robot_arm.RobotArm):
    """An xArm 7 from UFactory."""

    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "xarm7",
        joint_resting_configuration: Array = consts.JOINT_RESTING_CONFIGURATION,
        ik_point_link_name: Optional[str] = consts.IK_POINT_LINK_NAME,
        fixed_base: bool = consts.FIXED_BASE,
        max_joint_position_error: float = consts.MAX_JOINT_POSITION_ERROR,
    ) -> None:
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        # flags |= pb_client.URDF_USE_SELF_COLLISION  # type: ignore

        body_id = pb_client.load_urdf(
            str(consts.XARM7_URDF),
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

    def set_joint_angles(self, joint_angles: np.ndarray) -> None:
        pass
