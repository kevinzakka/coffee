import abc

from coffee import body
from coffee.client import BulletClient


class RobotHand(abc.ABC, body.NamedBody):
    """Robot hand base class."""

    @abc.abstractmethod
    def __init__(
        self,
        name: str,
        body_id: int,
        pb_client: BulletClient,
    ) -> None:
        """Constructor to be overridden by subclasses.

        Args:
        """
        super().__init__(name=name, body_id=body_id, pb_client=pb_client)

    @property
    @abc.abstractmethod
    def tool_center_point(self) -> str:
        """Tool center point link name."""
