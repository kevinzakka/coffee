"""Base class for sensors."""

import abc

import numpy as np

from coffee.client import BulletClient


class Sensor(abc.ABC):
    """Abstract base class for sensors.

    A sensor maps an underlying hidden state of the environment to a noisy measurement
    or observation.
    """

    def task_init(
        self,
        control_timestep_seconds: float,
        physics_timestep_seconds: float,
        physics_steps_per_control_step: int,
    ):
        """Setup the sensor for a task, before the environment has been setup.

        This is useful for sensors that need to know control or physics frequencies to
        function correctly.
        """

    @abc.abstractmethod
    def episode_init(
        self,
        pb_client: BulletClient,
        random_state: np.random.RandomState,
    ):
        """Setup the sensor for a new episode, after the environment has been reset.

        This is useful to reset any state the sensor has.
        """

    def close(self):
        """Clean up after we are done with the sensor.

        This is useful for real-world sensors that might need to close connections.
        """

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """The sensor's name."""
