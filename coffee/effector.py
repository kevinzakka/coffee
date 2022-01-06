"""Base class for effectors."""

import abc

import numpy as np
from dm_env import specs

from coffee.client import BulletClient


class Effector(abc.ABC):
    """Abstract base class for an effector.

    An effector provides an interface for an agent to interact in some way with the
    environment. An effector is defined by its action spec, a control method, and a
    prefix that marks the control components of the effector in the wider task action
    spec.
    """

    def close(self) -> None:
        """Clean up after we are done with the effector.

        This is useful for real-world effector that might need to close connections with
        the robot.
        """

    @abc.abstractmethod
    def episode_init(
        self,
        pb_client: BulletClient,
        random_state: np.random.RandomState,
    ) -> None:
        ...

    @abc.abstractmethod
    def action_spec(self, pb_client: BulletClient) -> specs.BoundedArray:
        ...

    @abc.abstractmethod
    def set_control(self, pb_client: BulletClient, command: np.ndarray) -> None:
        ...

    @property
    @abc.abstractmethod
    def prefix(self) -> str:
        pass
