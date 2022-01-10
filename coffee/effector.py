import abc

import numpy as np

from coffee.client import BulletClient


class Effector(abc.ABC):
    """Abstract base class for an effector."""

    def __init__(self, pb_client: BulletClient) -> None:
        """Effector constructor.

        Args:
            pb_client: The BulletClient instance.
        """
        self._pb_client = pb_client

    @abc.abstractmethod
    def set_control(self, command: np.ndarray) -> None:
        ...
