"""Base class for props."""

import contextlib
import pathlib
import tempfile
from typing import Any, Iterator, Mapping

from coffee import body


class Prop(body.NamedBody):
    """Base class for all props.

    A prop is any object within an environment that can be manipulated.
    """

    def disable_collisions(self) -> None:
        # TODO(kevin): Implement this method.
        raise NotImplementedError("Disabling collisions is not currently implemented.")


class TemplatedProp(Prop):
    """A prop created from a templated URDF file."""

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
