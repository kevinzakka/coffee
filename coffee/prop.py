"""Base class for props."""

import abc
import contextlib
import pathlib
import tempfile
from typing import Any, Iterator, Mapping, Optional

from dm_robotics.geometry import geometry

from coffee.client import BulletClient
from coffee.hints import Array


class Prop(abc.ABC):
    """Base class for all props.

    A prop is any object within an environment that can be manipulated.
    """

    def __init__(self, name: str, *args, **kwargs) -> None:
        """Prop constructor.

        Args:
            name: The unique name of this prop.
        """
        self._name = name
        self._body_id = -1

    def build(self, pb_client: BulletClient) -> None:
        """Build the prop.

        Args:
            pb_client: The BulletClient instance.

        Raises:
            RuntimeError: If the prop fails to build.
        """
        self._build(pb_client)

        if self._body_id < 0:
            raise RuntimeError(f"Failed to create prop {self._name}")

    @abc.abstractmethod
    def _build(self, pb_client: BulletClient) -> None:
        """Prop initialization method to be overridden by subclasses."""
        ...

    @property
    def body_id(self) -> int:
        return self._body_id

    @property
    def name(self) -> str:
        return self._name

    def set_pose(
        self,
        pb_client: BulletClient,
        position: Array,
        quaternion: Optional[Array] = None,
    ) -> None:
        """Sets the pose of the prop wrt the world frame."""
        if self.body_id < 0:
            raise RuntimeError("You must first build the prop.")

        pb_client.reset_body_state(
            body_id=self.body_id,
            pose=geometry.Pose(position=position, quaternion=quaternion),
        )

    def disable_collisions(self) -> None:
        if self.body_id < 0:
            raise RuntimeError("You must first build the prop.")

        # TODO(kevin): Implement this method.
        print("Disabling collisions is not currently implemented.")


class TemplatedProp(Prop):
    """A prop based on a templated URDF file that can be parameterized."""

    @contextlib.contextmanager
    def _fill_template(
        self,
        template_path: pathlib.Path,
        replace_dict: Mapping[str, Any],
    ) -> Iterator[str]:
        """Reads a URDF template file and replaces the template variables.

        Args:
            template_path: The path to the URDF template file.
            replace_dict: A dictionary mapping template variable names to replacement
                values.

        Returns:
            Path to a temporary file containing the filled URDF.
        """
        # Read the template file.
        with open(template_path, "r") as fp:
            data = fp.read()

        # Replace the template variables.
        data = data.format(**replace_dict)

        # Write the filled URDF to a temporary file.
        with tempfile.NamedTemporaryFile(
            suffix=".urdf",
            dir=template_path.parent,
            delete=True,
        ) as tf:
            tf.write(data.encode("utf-8"))
            tf.flush()
            yield tf.name
