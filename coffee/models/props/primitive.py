import abc
from typing import Any, Mapping, Optional, Tuple

import pybullet as p

import coffee.models.props.constants as consts
from coffee import prop
from coffee.client import BulletClient


class Primitive(prop.Prop):
    """A primitive PyBullet geom prop."""

    @abc.abstractmethod
    def __init__(
        self,
        name: str,
        pb_client: BulletClient,
        collision_args: Mapping[str, Any],
        visual_args: Mapping[str, Any],
        mass: float = 0.0,
        color: Optional[Tuple[float, float, float, float]] = None,
    ) -> None:
        """Primitive constructor.

        Args:
            name: The name of the primitive.
            pb_client: The BulletClient instance.
            collision_args: The arguments to pass to the primitive's collision creation
                function.
            visual_args: The arguments to pass to the primitive's visual creation
                function.
            mass: The mass of the primitive. If 0, the primitive is static.
            color: The RGBA color of the primitive. If None, the primitive is white.
        """
        v_id = pb_client.createVisualShape(**visual_args, rgbaColor=color)
        c_id = pb_client.createCollisionShape(**collision_args)
        multi_body_kwargs = dict(
            baseMass=mass,
            baseVisualShapeIndex=v_id,
            baseCollisionShapeIndex=c_id,
        )
        body_id = pb_client.createMultiBody(**multi_body_kwargs)

        super().__init__(name=name, body_id=body_id, pb_client=pb_client)


class Sphere(Primitive):
    """A sphere prop."""

    def __init__(
        self,
        pb_client: BulletClient,
        radius: float = consts._SPHERE_RADIUS,
        mass: float = consts._SPHERE_MASS,
        color: Optional[Tuple[float, float, float, float]] = None,
        name: str = "sphere",
    ) -> None:

        collision_args = dict(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
        )
        visual_args = dict(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
        )

        super().__init__(
            name=name,
            pb_client=pb_client,
            collision_args=collision_args,
            visual_args=visual_args,
            mass=mass,
            color=color,
        )


class Box(Primitive):
    """A box prop."""

    def __init__(
        self,
        pb_client: BulletClient,
        half_extents: Tuple[float, float, float] = (
            consts._BOX_DEPTH / 2,
            consts._BOX_WIDTH / 2,
            consts._BOX_HEIGHT / 2,
        ),
        mass: float = consts._BOX_MASS,
        color: Optional[Tuple[float, float, float, float]] = None,
        name: str = "box",
    ) -> None:

        collision_args = dict(
            shapeType=p.GEOM_BOX,
            halfExtents=half_extents,
        )
        visual_args = dict(
            shapeType=p.GEOM_BOX,
            halfExtents=half_extents,
        )

        super().__init__(
            name=name,
            pb_client=pb_client,
            collision_args=collision_args,
            visual_args=visual_args,
            mass=mass,
            color=color,
        )


class Cylinder(Primitive):
    """A cylinder prop."""

    def __init__(
        self,
        pb_client: BulletClient,
        radius: float = consts._CYLINDER_RADIUS,
        length: float = consts._CYLINDER_LENGTH,
        mass: float = consts._CYLINDER_MASS,
        color: Optional[Tuple[float, float, float, float]] = None,
        name: str = "cylinder",
    ) -> None:

        collision_args = dict(
            shapeType=p.GEOM_CYLINDER,
            radius=radius,
            height=length,
        )
        visual_args = dict(
            shapeType=p.GEOM_CYLINDER,
            radius=radius,
            length=length,
        )

        super().__init__(
            name=name,
            pb_client=pb_client,
            collision_args=collision_args,
            visual_args=visual_args,
            mass=mass,
            color=color,
        )


class Capsule(Primitive):
    """A capsule prop."""

    def __init__(
        self,
        pb_client: BulletClient,
        radius: float = consts._CAPSULE_RADIUS,
        length: float = consts._CAPSULE_LENGTH,
        mass: float = consts._CAPSULE_RADIUS,
        color: Optional[Tuple[float, float, float, float]] = None,
        name: str = "capsule",
    ) -> None:

        collision_args = dict(
            shapeType=p.GEOM_CAPSULE,
            radius=radius,
            height=length,
        )
        visual_args = dict(
            shapeType=p.GEOM_CAPSULE,
            radius=radius,
            length=length,
        )

        super().__init__(
            name=name,
            pb_client=pb_client,
            collision_args=collision_args,
            visual_args=visual_args,
            mass=mass,
            color=color,
        )
