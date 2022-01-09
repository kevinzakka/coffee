from coffee.client import BulletClient
from coffee.models.end_effectors.robot_hands import robot_hand
from coffee.models.end_effectors.robot_hands import schunk_wsg_50_constants as consts

_DEFAULT_FRICTION = (1.0, 1.0, 0.0001)
_DARK_COLOR = (0.1, 0.1, 0.1, 1)
_GRAY_COLOR = (0.5, 0.5, 0.5, 1)
_LINK_NAME_TO_COLOR = {
    # "base_link": _GRAY_COLOR,
    "gripper_left": _DARK_COLOR,
    "gripper_right": _DARK_COLOR,
    "finger_left": _DARK_COLOR,
    "finger_right": _DARK_COLOR,
}
_MIMIC_PARENT_LINK_NAME = "gripper_right"
_MIMIC_CHILDREN_LINK_NAMES = ("gripper_left",)
_MIMIC_COEFFICIENTS = {
    "gripper_left": -1,
}


class SchunkWsg50(robot_hand.RobotHand):
    """Schunk WSG 50 2-finger parallel gripper."""

    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "schunk_wsg_50",
        use_realistic_friction: bool = True,
        scaling: float = 1.0,
    ) -> None:
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        flags |= pb_client.URDF_USE_SELF_COLLISION  # type: ignore

        body_id = pb_client.load_urdf(
            str(consts.SCHUNK_WSG_50_URDF),
            flags=flags,
            scaling=scaling,
        )

        super().__init__(
            name=name,
            body_id=body_id,
            pb_client=pb_client,
        )

        self._set_physics_properties(use_realistic_friction)
        self._color_gripper()

        self._mimic_parent_id = self.joints.get_joint_index_from_link_name(
            _MIMIC_PARENT_LINK_NAME
        )
        self._mimic_children_ids = [
            self.joints.get_joint_index_from_link_name(link_name)
            for link_name in _MIMIC_CHILDREN_LINK_NAMES
        ]

    def _set_physics_properties(self, use_realistic_friction: bool) -> None:
        """Sets physics-related properties."""
        if use_realistic_friction:
            for i in range(self.joints.dof):
                self.pb_client.changeDynamics(
                    bodyUniqueId=self.body_id,
                    linkIndex=i,
                    lateralFriction=_DEFAULT_FRICTION[0],
                    spinningFriction=_DEFAULT_FRICTION[1],
                    rollingFriction=_DEFAULT_FRICTION[2],
                    frictionAnchor=True,
                )

    def _color_gripper(self) -> None:
        """Modifes the color of the gripper."""
        for link_name, link_color in _LINK_NAME_TO_COLOR.items():
            self.pb_client.changeVisualShape(
                objectUniqueId=self.body_id,
                linkIndex=self.joints.get_joint_index_from_link_name(link_name),
                textureUniqueId=-1,
                rgbaColor=link_color,
            )

    def _set_constraints(self) -> None:
        qpos = self.pb_client.getJointState(self.body_id, self._mimic_parent_id)[0]

        self.pb_client.setJointMotorControlArray(
            bodyIndex=self.body_id,
            jointIndices=self._mimic_children_ids,
            controlMode=self.pb_client.POSITION_CONTROL,
            targetPositions=[qpos * coeff for coeff in _MIMIC_COEFFICIENTS.values()],
            forces=[100] * len(self._mimic_children_ids),
            positionGains=[2 * 0.1] * len(self._mimic_children_ids),
        )

    def tool_center_point(self):
        pass
