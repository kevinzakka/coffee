from coffee.client import BulletClient
from coffee.models.end_effectors.robot_hands import rg2_constants as consts
from coffee.models.end_effectors.robot_hands import robot_hand

_DEFAULT_FRICTION = (1.0, 1.0, 0.0001)
_DARK_COLOR = (0.1, 0.1, 0.1, 1)
_GRAY_COLOR = (0.5, 0.5, 0.5, 1)
_LINK_NAME_TO_COLOR = {
    "left_inner": _GRAY_COLOR,
    "left_outer": _GRAY_COLOR,
    "left_finger": _DARK_COLOR,
    "right_inner": _GRAY_COLOR,
    "right_outer": _GRAY_COLOR,
    "right_finger": _DARK_COLOR,
}
_MIMIC_PARENT_LINK_NAME = "left_outer"
_MIMIC_CHILDREN_LINK_NAMES = (
    "left_inner",
    "left_finger",
    "right_inner",
    "right_outer",
    "right_finger",
)
_MIMIC_COEFFICIENTS = {
    "left_inner": 1,
    "left_finger": -1,
    "right_inner": -1,
    "right_outer": -1,
    "right_finger": 1,
}
_TCP_LINK_NAME = "pinch_site"


class RG2(robot_hand.RobotHand):
    """OnRobot RG2 2-finger gripper."""

    def __init__(
        self,
        pb_client: BulletClient,
        name: str = "rg2",
        use_realistic_friction: bool = True,
        scaling: float = 1.0,
        enable_self_collision: bool = consts.ENABLE_SELF_COLLISION,
    ) -> None:
        flags = pb_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        if enable_self_collision:
            flags |= pb_client.URDF_USE_SELF_COLLISION  # type: ignore

        body_id = pb_client.load_urdf(
            str(consts.RG2_URDF),
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

    @property
    def tool_center_point(self) -> str:
        return _TCP_LINK_NAME
